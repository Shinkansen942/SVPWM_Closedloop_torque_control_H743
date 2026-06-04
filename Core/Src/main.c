/* USER CODE BEGIN Header */
/*
  TODO:
  UART Control (low)
  Get Mot temp conversion ratio
  SDMMC Log  
  Encoder Error
  RTC
  DONE:
  SDMMC Test with FATFS without MDMA
  ADC DMA
  Bypass SD Card detetion in BSP_PlatformIsDetected
  Invalidate DCache after ACD finish conversion
  OCP OTP Error
  SDMMC with FATFS and IDMA
  Change CAN send data
*/

// #pragma GCC optimize("O2")
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "fdcan.h"
#include "i2c.h"
#include "mdma.h"
#include "rtc.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
// #include "usbd_cdc_if.h"
#include "string.h"
// #include "as5048a.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "current_sense.h"
#include "inverter_state.h"
#include "logger.h"
// #include "stdio.h"
// #include "stm32h7xx_hal_tim_ex.h"
#include "time.h"
#include "can_app.h"
#include "foc_loop.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
float MCU_MapValue(uint16_t in_value, float in_min, float in_max, float out_min, float out_max);
float MCU_TemperatureCalculate(uint16_t ts_data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile int code_ver = 5;

// Communication & UART
__attribute__((section("._RAM_Area"))) static char tdata[500];
extern int CAN_Timer;
volatile int isSent = 1;

// Motor Control & FOC variables
#ifdef OPEN_LOOP_SPEED
float open_loop_rpm_var = OPEN_LOOP_RPM;
float last_angle = 0.0f;
#endif
motor_params_t motor = {
    /* const fields — set once, compiler-enforced immutable */
    .Rs = 0.126f,                    // Stator resistance  (Ohm)
    .Ld = 2.49f * 0.0001f,          // D-axis inductance  (H)
    .Lq = 3.8f * 0.0001f,           // Q-axis inductance  (H)
    .flux_linkage_m = 4.75f * 0.01f, // PM flux linkage   (Wb)
    .electrical_constant = 0.031f,   // Back-EMF constant  (V/(rad/s))
    .max_current = 70,               // Phase current limit (A)
    .pole_pairs = 1,
    .dir = 1,                        // +1 CCW, -1 CW
    .Ts = (float)1/FREQ,

    /* runtime-writable fields */
    .zero_electric_angle = ZERO_ELECTRIC_ANGLE,
    .voltage_power_supply = 440,
    .voltage_limit = 440,
};

foc_state_t foc = {
    .max_ramp = 1/FREQ/RAMP_TIME_DERATE,
    .period = DEF_CCR,
    .freq = FREQ,
};

// Current filter
lpf_t filter_current_Iq= {.Tf=QTF,.y_prev=0.0f}; //Tf=20ms
lpf_t filter_current_Id= {.Tf=DTF,.y_prev=0.0f}; //Tf=20ms
lpf_t filter_current_Iabc[3] = {{.Tf = ABCTF,.y_prev=0.0f},{.Tf = ABCTF,.y_prev=0.0f},{.Tf = ABCTF,.y_prev=0.0f}}; //Tf=2ms
lpf_t filter_current_DC_Iabc[3] = {{.Tf = DCTF,.y_prev=0.0f},{.Tf = DCTF,.y_prev=0.0f},{.Tf = DCTF,.y_prev=0.0f}}; //Tf=2ms
lpf_t filter_RPM = {.Tf=RPMTF,.y_prev=0.0f};
lpf_t filter_Idfw = {.Tf=FWTF,.y_prev=0.0f};
lpf_t filter_report_torque = {.Tf=0.008f,.y_prev=0.0f}; //Tf=100ms

// PID controller
pidc_t pid_controller_current_Iq = {.P=QKP,.I=QKI,.D=QKD,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_Id = {.P=DKP,.I=DKI,.D=DKD,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_OCP = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_Ia = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_Iabc[3] = {
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0},
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0},
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0}
};

// Temperature & Calibration
static float ts_cal1;
static float ts_cal2;
int16_t Mot_Conv[1024] = {0};
float Mot_Curr = MOT_CURR;
int16_t Inv_Conv[1024] = {-750,-696,-608,-553,-512,-479,-452,-428,-407,-388,-370,-355,-340,-326,-314,-301,-290,-279,-269,-259,-250,-241,-232,-224,-216,-208,-200,-193,-186,-179,-172,-166,-160,-153,-147,-142,-136,-130,-125,-119,-114,-109,-104,-99,-94,-89,-84,-80,-75,-71,-66,-62,-58,-53,-49,-45,-41,-37,-33,-29,-26,-22,-18,-14,-11,-7,-4,0,2,6,9,13,16,19,22,26,29,32,35,38,41,44,47,50,53,56,59,62,65,68,71,73,76,79,82,84,87,90,92,95,98,100,103,105,108,110,113,115,118,120,123,125,128,130,132,135,137,139,142,144,146,149,151,153,155,158,160,162,164,167,169,171,173,175,177,179,182,184,186,188,190,192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,231,233,235,237,239,241,243,245,246,248,250,252,254,256,257,259,261,263,265,266,268,270,272,273,275,277,279,280,282,284,286,287,289,291,292,294,296,297,299,301,302,304,306,307,309,311,312,314,316,317,319,321,322,324,325,327,329,330,332,333,335,337,338,340,341,343,345,346,348,349,351,352,354,355,357,359,360,362,363,365,366,368,369,371,372,374,375,377,378,380,381,383,384,386,387,389,390,392,393,395,396,398,399,401,402,404,405,407,408,409,411,412,414,415,417,418,420,421,422,424,425,427,428,430,431,432,434,435,437,438,440,441,442,444,445,447,448,449,451,452,454,455,456,458,459,461,462,463,465,466,468,469,470,472,473,474,476,477,479,480,481,483,484,485,487,488,490,491,492,494,495,496,498,499,501,502,503,505,506,507,509,510,511,513,514,515,517,518,519,521,522,523,525,526,528,529,530,532,533,534,536,537,538,540,541,542,544,545,546,548,549,550,552,553,554,556,557,558,560,561,562,564,565,566,568,569,570,572,573,574,576,577,578,580,581,582,583,585,586,587,589,590,591,593,594,595,597,598,599,601,602,603,605,606,607,609,610,611,613,614,615,617,618,619,621,622,623,625,626,627,629,630,631,633,634,635,636,638,639,640,642,643,644,646,647,648,650,651,652,654,655,656,658,659,660,662,663,664,666,667,668,670,671,672,674,675,677,678,679,681,682,683,685,686,687,689,690,691,693,694,695,697,698,699,701,702,703,705,706,708,709,710,712,713,714,716,717,718,720,721,723,724,725,727,728,729,731,732,733,735,736,738,739,740,742,743,745,746,747,749,750,751,753,754,756,757,758,760,761,763,764,765,767,768,770,771,772,774,775,777,778,780,781,782,784,785,787,788,789,791,792,794,795,797,798,800,801,802,804,805,807,808,810,811,813,814,815,817,818,820,821,823,824,826,827,829,830,832,833,835,836,838,839,840,842,843,845,846,848,849,851,852,854,856,857,859,860,862,863,865,866,868,869,871,872,874,875,877,878,880,882,883,885,886,888,889,891,893,894,896,897,899,900,902,904,905,907,908,910,912,913,915,916,918,920,921,923,925,926,928,929,931,933,934,936,938,939,941,943,944,946,948,949,951,953,954,956,958,959,961,963,965,966,968,970,971,973,975,977,978,980,982,984,985,987,989,991,992,994,996,998,1000,1001,1003,1005,1007,1009,1010,1012,1014,1016,1018,1020,1021,1023,1025,1027,1029,1031,1033,1034,1036,1038,1040,1042,1044,1046,1048,1050,1052,1054,1055,1057,1059,1061,1063,1065,1067,1069,1071,1073,1075,1077,1079,1081,1083,1085,1087,1089,1091,1093,1095,1097,1100,1102,1104,1106,1108,1110,1112,1114,1116,1118,1121,1123,1125,1127,1129,1131,1134,1136,1138,1140,1142,1145,1147,1149,1151,1154,1156,1158,1160,1163,1165,1167,1169,1172,1174,1176,1179,1181,1184,1186,1188,1191,1193,1195,1198,1200,1203,1205,1208,1210,1213,1215,1218,1220,1223,1225,1228,1230,1233,1235,1238,1241,1243,1246,1248,1251,1254,1256,1259,1262,1264,1267,1270,1273,1275,1278,1281,1284,1287,1289,1292,1295,1298,1301,1304,1307,1310,1312,1315,1318,1321,1324,1327,1330,1333,1337,1340,1343,1346,1349,1352,1355,1358,1362,1365,1368,1371,1375,1378,1381,1385,1388,1391,1395,1398,1402,1405,1409,1412,1416,1419,1423,1426,1430,1434,1437,1441,1445,1448,1452,1456,1460,1464,1468,1471,1475,1479,1483,1487,1491,1496,1500,1504,1508,1512,1516,1521,1525,1529,1534,1538,1542,1547,1551,1556,1561,1565,1570,1575,1579,1584,1589,1594,1599,1604,1609,1614,1619,1624,1629,1635,1640,1645,1651,1656,1662,1667,1673,1678,1684,1690,1696,1702,1708,1714,1720,1726,1733,1739,1745,1752,1758,1765,1772,1779,1785,1792,1800,1807,1814,1821,1829,1836,1844,1852,1860,1868,1876,1884,1892,1901,1909,1918,1927,1936,1945,1954,1964,1973,1983,1993,2003,2014,2024,2035,2046,2057,2068,2080,2091,2103,2116,2128,2141,2154,2168,2181,2195,2210,2224,2239,2255,2271,2287,2304,2321,2338,2356,2375,2394,2414,2435,2456,2478,2501,2524,2549,2574,2600,2628,2657,2686,2718,2750,2785,2821,2859,2899,2942,2987,3035,3087,3142,3202,3266,3336,3412,3497,3590,3695,3814,3951,4111,4301,4534,4830,5227,5804,6778,9092};
int16_t MCU_Conv[1024] = {0};

// Protection & Telemetry & Moving RMS Buffers
protection_t prot = {0};
telemetry_t telem = {0};

// Debug & Timing
#ifdef TIMING
int max_time = 0;
int min_time = INT32_MAX;
int prev_time = 0;
int max_btw = 0;
int max_sprint = 0;
int max_sdwrite = 0;
int sd_td;
uint32_t loop_time;
#endif

void UART_TX_Send(UART_HandleTypeDef *huart,const char *format,...)
{
  va_list list;
  va_start(list,format);
  vsnprintf(tdata,sizeof(tdata),format,list);
  if(isSent)
  {
    HAL_UART_Transmit_DMA(huart,(const uint8_t*)tdata,sizeof(tdata));
    isSent = 0;
  }
  va_end(list);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // --- Version & PID limits ---
  code_ver = CODE_VER;
	// uint8_t wtext[] = "This is STM32 working with FatFs\n"; /* File write buffer */
  pid_controller_current_Ia.limit = motor.voltage_limit;
  pid_controller_current_Id.limit = motor.voltage_limit;
  pid_controller_current_Iq.limit = motor.voltage_limit;
  pid_controller_current_OCP.limit = motor.voltage_limit;
  // uint8_t wlooptext[] = "This is STM32 working with FatFs in main loop\n"; /* File write buffer */
  // log_buf[0][0].LGSTATE = 0;

  // --- Build temperature lookup tables (Mot_Conv, Inv_Conv) ---
  for (size_t i = 0; i < 1024; i++)
  {
    float voltage = (float)(3.3*i/1024);
    // Mot_Conv[i] = (int16_t)10*(voltage/Mot_Curr/3.795-1000/3.795);
    Inv_Conv[i] = (int16_t)10*(float)((float)(2000*voltage-3300)/((float)(3.3-voltage)*3.8505));
    Mot_Conv[i] = (int16_t)10*((float)(0.5*(3300*i/1024))/Mot_Curr/3.795-1000/3.795);
    // if (Mot_Conv[i] < 0)
    // {
    //   Mot_Conv[i] = 0;
    // }
  }

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_MDMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_RTC_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  // --- LED & motor driver init ---
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED_D12_GPIO_Port,LED_D12_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_SD_GPIO_Port, LED_SD_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_TIM_GPIO_Port, LED_TIM_Pin, GPIO_PIN_RESET);

  // --- MCU temperature calibration (read factory cal + build MCU_Conv table) ---
  /* 0x1FF1E820 Calibration ADC value at 30 °C = 0x2fc0, 12224 */
  ts_cal1 = (float) *(uint16_t*) (TEMPSENSOR_CAL1_ADDR);
  /* 0x1FF1E840 Calibration ADC value at 110 °C = 0x3cb4, 15540 */
  ts_cal2 = (float) *(uint16_t*) (TEMPSENSOR_CAL2_ADDR);
  for (size_t i = 0; i < 1024; i++)
  {
    MCU_Conv[i] = (int16_t) roundf(MCU_TemperatureCalculate(i<<6)*10);
  }

  // --- ADC calibration ---
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
  HAL_Delay(100);

  // --- Start utility timers (htim5: ISR timing, htim2: SD write interval) ---
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim2);

  // --- SD card mount ---
  sd_logger_init();

  // --- ADC DMA start ---
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMA_ADC1_arr, 4);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)DMA_ADC2_arr, 4);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)DMA_ADC3_arr, 6);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel0_dma1_stream2_tc_0,(uint32_t)tmp_DMA_ADC1_arr,(uint32_t)DMA_ADC1_arr,8,1);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel1_dma1_stream1_tc_0,(uint32_t)tmp_DMA_ADC2_arr,(uint32_t)DMA_ADC2_arr,8,1);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel2_dma1_stream4_tc_0,(uint32_t)tmp_DMA_ADC3_arr,(uint32_t)DMA_ADC3_arr,12,1);
  // HAL_ADC_Start_IT(&hadc3);

  // --- PWM start + current offset calibration ---
  // HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_Delay(1000);
  calibrateOffsets(foc.current_offset, DMA_ADC1_arr);
  // HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);

  // --- Wait for gate driver ready (optional) ---
  #ifdef WAIT_GATE_READY
  while (HAL_GPIO_ReadPin(GATE_Ready_GPIO_Port,GATE_Ready_Pin) != GPIO_PIN_SET)
  {
    HAL_Delay(1);
  }
  #endif

  // --- Zero electrical angle calibration (optional) ---
  #ifdef CAL_ZERO_ANGLE
  float angle_integrate = 0.0f;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
  setPhaseVoltage(25,0,_electricalAngle(M_PI*1.5f,motor.pole_pairs),TIM1,0,0,0);
  for (size_t i = 0; i < 2000; i++)
  {
    Get_Encoder_Angle(DMA_ADC2_arr, &foc.angle_now, NULL, NULL);
    HAL_Delay(10);
    SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr,sizeof(DMA_ADC2_arr));
    if (i >= 1500)
    {
      angle_integrate += foc.angle_now;
    }
  }

  // uint16_t read_raw=read(&hspi1, SPI1_CSn_GPIO_Port,SPI1_CSn_Pin,AS5048A_ANGLE);
  float raw_angle;
  SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr,sizeof(DMA_ADC2_arr));
  Get_Encoder_Angle(DMA_ADC2_arr,&raw_angle,NULL,NULL);
  raw_angle = angle_integrate/500.0f;
  motor.zero_electric_angle=_electricalAngle(raw_angle,motor.pole_pairs);
  setPhaseVoltage(0,0,_electricalAngle(M_PI*1.5f,motor.pole_pairs),TIM1,0,0,0);
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  #endif

  // --- CAN bus init ---
  UART_TX_Send(&huart1, "zero_electric_angle: %i \n", (int) floor(motor.zero_electric_angle/M_PI*180));
  Config_Fdcan1();

  // while (!got_date)
  // {
  //   HAL_Delay(10);
  // }

  // --- Open SD log file ---
  sd_logger_open_file();

  // --- Enter READY state, start FOC & protection timer ISR ---
  HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
  inverter_state = STATE_READY;
  error_state = ERROR_NONE;
  // INV_Statustypedef last_state = inverter_state;

  #ifdef TIMING
  prev_time = __HAL_TIM_GET_COUNTER(&htim5);
  #endif

  HAL_GPIO_WritePin(LED_SD_GPIO_Port,LED_SD_Pin,GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim1);  // FOC control loop ISR
  HAL_TIM_Base_Start_IT(&htim3);  // protection timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // continue;
    sd_logger_run();      // double-buffer write + file rotation
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 32;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SDMMC;
  PeriphClkInitStruct.PLL2.PLL2M = 3;
  PeriphClkInitStruct.PLL2.PLL2N = 150;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 20;
  PeriphClkInitStruct.PLL2.PLL2R = 4;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float MCU_MapValue(uint16_t in_value, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(in_value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}

float MCU_TemperatureCalculate(uint16_t ts_data)
{

//  return (80 * (ts_data - ts_cal1)) / (ts_cal2 - ts_cal1) + 30;

  return MCU_MapValue(ts_data, ts_cal1, ts_cal2, TEMPSENSOR_CAL1_TEMP, TEMPSENSOR_CAL2_TEMP);
  return 1;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
