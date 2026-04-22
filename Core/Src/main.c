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
#include "arm_math.h"
#include "FOC.h"
#include "can_app.h"
#include "state_machine.h"
#include "foc_loop.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif
#define MAX_ANGLE_VALUE 4096
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define CODE_VER 0x3
// #define NUM_SECTIONS 7 // Number of sections in the Biquad filter
#define NUM_TAPS 558
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

char data[500];
__attribute__((section("._RAM_Area"))) static char tdata[500];

__attribute__((section("._RAM_Area"))) FATFS SDFatFS_RAM;  /* File system object for SD card logical drive */
__attribute__((section("._RAM_Area"))) FIL MyFile;     /* File object */

//Logging Buffers
__attribute__((section("._RAM_Area"))) logger_t log_buf[2][3600];
// __attribute__((section("._RAM_Area"))) logger_t log_test[2][2];

uint8_t wr_log_buf_num = 0;
uint16_t wr_log_index = 0;
RTC_DateTypeDef log_date;
RTC_TimeTypeDef log_time;
uint8_t last_sec = 0;
uint16_t log_subsec = 0;

// Moving RMS Buffers
protection_t prot = {0};
int16_t RMS_buf[3][10000] = {{0}};
uint32_t RMS_sum[3] = {0};
uint16_t indexRMS = 0;


// __attribute__((section("._RAM_D2_Area")))
uint8_t oc_buf[HW_OC_TIME] = {0};
uint16_t oc_index = 0;
uint16_t oc_sum = 0;

// __attribute__((section("._RAM_D2_Area")))
uint8_t soft_oc_buf[SOFT_OC_TIME] = {0};
uint16_t soft_oc_index = 0;
uint16_t soft_oc_sum = 0;

// __attribute__((section("._RAM_D2_Area")))
uint8_t enc_buf[ENC_TIME] = {0};
uint16_t enc_index = 0;
uint16_t enc_sum = 0;

telemetry_t telem = {0};

#ifdef OPEN_LOOP_SPEED
float open_loop_rpm_var = OPEN_LOOP_RPM;
float last_angle = 0.0f;
#endif

// FOC variables
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

const float ACAPLSB = -0.1031436f;   // ACAPLSB = 3.3/15.626e-3/adc1_range
const float derate_a = 1/(DERATE_START-DERATE_END);
const float derate_b = DERATE_END/(DERATE_START-DERATE_END);

int indexLED=0;
int indexHeartbeat=0;
int indexStatus=0;
int indexTimer = 0;
uint32_t indexMusic = 0;
uint32_t indexMusicTs = 0;
uint32_t prev_new_file = 0;
int prev_sd = 0;
int prevWhileTest = 0;
uint8_t got_date = 0;
uint8_t last_got_date = 0;

const float torque_constant = 0.291f; //Nm/A
const float max_torque = 25;
int16_t Mot_Conv[1024] = {0};
float Mot_Curr = MOT_CURR;
int16_t Inv_Conv[1024] = {-750,-696,-608,-553,-512,-479,-452,-428,-407,-388,-370,-355,-340,-326,-314,-301,-290,-279,-269,-259,-250,-241,-232,-224,-216,-208,-200,-193,-186,-179,-172,-166,-160,-153,-147,-142,-136,-130,-125,-119,-114,-109,-104,-99,-94,-89,-84,-80,-75,-71,-66,-62,-58,-53,-49,-45,-41,-37,-33,-29,-26,-22,-18,-14,-11,-7,-4,0,2,6,9,13,16,19,22,26,29,32,35,38,41,44,47,50,53,56,59,62,65,68,71,73,76,79,82,84,87,90,92,95,98,100,103,105,108,110,113,115,118,120,123,125,128,130,132,135,137,139,142,144,146,149,151,153,155,158,160,162,164,167,169,171,173,175,177,179,182,184,186,188,190,192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,231,233,235,237,239,241,243,245,246,248,250,252,254,256,257,259,261,263,265,266,268,270,272,273,275,277,279,280,282,284,286,287,289,291,292,294,296,297,299,301,302,304,306,307,309,311,312,314,316,317,319,321,322,324,325,327,329,330,332,333,335,337,338,340,341,343,345,346,348,349,351,352,354,355,357,359,360,362,363,365,366,368,369,371,372,374,375,377,378,380,381,383,384,386,387,389,390,392,393,395,396,398,399,401,402,404,405,407,408,409,411,412,414,415,417,418,420,421,422,424,425,427,428,430,431,432,434,435,437,438,440,441,442,444,445,447,448,449,451,452,454,455,456,458,459,461,462,463,465,466,468,469,470,472,473,474,476,477,479,480,481,483,484,485,487,488,490,491,492,494,495,496,498,499,501,502,503,505,506,507,509,510,511,513,514,515,517,518,519,521,522,523,525,526,528,529,530,532,533,534,536,537,538,540,541,542,544,545,546,548,549,550,552,553,554,556,557,558,560,561,562,564,565,566,568,569,570,572,573,574,576,577,578,580,581,582,583,585,586,587,589,590,591,593,594,595,597,598,599,601,602,603,605,606,607,609,610,611,613,614,615,617,618,619,621,622,623,625,626,627,629,630,631,633,634,635,636,638,639,640,642,643,644,646,647,648,650,651,652,654,655,656,658,659,660,662,663,664,666,667,668,670,671,672,674,675,677,678,679,681,682,683,685,686,687,689,690,691,693,694,695,697,698,699,701,702,703,705,706,708,709,710,712,713,714,716,717,718,720,721,723,724,725,727,728,729,731,732,733,735,736,738,739,740,742,743,745,746,747,749,750,751,753,754,756,757,758,760,761,763,764,765,767,768,770,771,772,774,775,777,778,780,781,782,784,785,787,788,789,791,792,794,795,797,798,800,801,802,804,805,807,808,810,811,813,814,815,817,818,820,821,823,824,826,827,829,830,832,833,835,836,838,839,840,842,843,845,846,848,849,851,852,854,856,857,859,860,862,863,865,866,868,869,871,872,874,875,877,878,880,882,883,885,886,888,889,891,893,894,896,897,899,900,902,904,905,907,908,910,912,913,915,916,918,920,921,923,925,926,928,929,931,933,934,936,938,939,941,943,944,946,948,949,951,953,954,956,958,959,961,963,965,966,968,970,971,973,975,977,978,980,982,984,985,987,989,991,992,994,996,998,1000,1001,1003,1005,1007,1009,1010,1012,1014,1016,1018,1020,1021,1023,1025,1027,1029,1031,1033,1034,1036,1038,1040,1042,1044,1046,1048,1050,1052,1054,1055,1057,1059,1061,1063,1065,1067,1069,1071,1073,1075,1077,1079,1081,1083,1085,1087,1089,1091,1093,1095,1097,1100,1102,1104,1106,1108,1110,1112,1114,1116,1118,1121,1123,1125,1127,1129,1131,1134,1136,1138,1140,1142,1145,1147,1149,1151,1154,1156,1158,1160,1163,1165,1167,1169,1172,1174,1176,1179,1181,1184,1186,1188,1191,1193,1195,1198,1200,1203,1205,1208,1210,1213,1215,1218,1220,1223,1225,1228,1230,1233,1235,1238,1241,1243,1246,1248,1251,1254,1256,1259,1262,1264,1267,1270,1273,1275,1278,1281,1284,1287,1289,1292,1295,1298,1301,1304,1307,1310,1312,1315,1318,1321,1324,1327,1330,1333,1337,1340,1343,1346,1349,1352,1355,1358,1362,1365,1368,1371,1375,1378,1381,1385,1388,1391,1395,1398,1402,1405,1409,1412,1416,1419,1423,1426,1430,1434,1437,1441,1445,1448,1452,1456,1460,1464,1468,1471,1475,1479,1483,1487,1491,1496,1500,1504,1508,1512,1516,1521,1525,1529,1534,1538,1542,1547,1551,1556,1561,1565,1570,1575,1579,1584,1589,1594,1599,1604,1609,1614,1619,1624,1629,1635,1640,1645,1651,1656,1662,1667,1673,1678,1684,1690,1696,1702,1708,1714,1720,1726,1733,1739,1745,1752,1758,1765,1772,1779,1785,1792,1800,1807,1814,1821,1829,1836,1844,1852,1860,1868,1876,1884,1892,1901,1909,1918,1927,1936,1945,1954,1964,1973,1983,1993,2003,2014,2024,2035,2046,2057,2068,2080,2091,2103,2116,2128,2141,2154,2168,2181,2195,2210,2224,2239,2255,2271,2287,2304,2321,2338,2356,2375,2394,2414,2435,2456,2478,2501,2524,2549,2574,2600,2628,2657,2686,2718,2750,2785,2821,2859,2899,2942,2987,3035,3087,3142,3202,3266,3336,3412,3497,3590,3695,3814,3951,4111,4301,4534,4830,5227,5804,6778,9092};
int16_t MCU_Conv[1024] = {0};
const float DCVPLSB = 0.00897;     // DCVPLSB = 451*3.3/adc3_range
const float DCAPLSB = 0.0402930f;   // DCAPLSB = 3.3/20e-3/adc1_range

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

int max_sd_buf = 0;

__attribute__((section("._ADC1_Area"))) uint16_t DMA_ADC1_arr[4] = {0};
__attribute__((section("._ADC2_Area"))) uint16_t DMA_ADC2_arr[4] = {0};
__attribute__((section("._ADC3_Area"))) uint16_t DMA_ADC3_arr[6] = {0};

//Temperature Constants
float ts_cal1;
float ts_cal2;

extern int CAN_Timer;
int isSent = 1;

const char TestFPath[] = {"Test.bin"};
char TextFPath[80];

lpf_t filter_current_Iq= {.Tf=QTF,.y_prev=0.0f}; //Tf=20ms
lpf_t filter_current_Id= {.Tf=DTF,.y_prev=0.0f}; //Tf=20ms
lpf_t filter_current_Iabc[3] = {{.Tf = ABCTF,.y_prev=0.0f},{.Tf = ABCTF,.y_prev=0.0f},{.Tf = ABCTF,.y_prev=0.0f}}; //Tf=2ms
lpf_t filter_crrent_DC_Iabc[3] = {{.Tf = DCTF,.y_prev=0.0f},{.Tf = DCTF,.y_prev=0.0f},{.Tf = DCTF,.y_prev=0.0f}}; //Tf=2ms
lpf_t filter_RPM = {.Tf=RPMTF,.y_prev=0.0f};
lpf_t filter_Idfw = {.Tf=FWTF,.y_prev=0.0f};
lpf_t filter_report_torque = {.Tf=0.008f,.y_prev=0.0f}; //Tf=100ms
pidc_t pid_controller_current_Iq = {.P=QKP,.I=QKI,.D=QKD,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_Id = {.P=DKP,.I=DKI,.D=DKD,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_OCP = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_Ia = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_Iabc[3] = {
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0},
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0},
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0}
};

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
  code_ver = CODE_VER;
  volatile FRESULT res;                                 /* FatFs function common result code */
	uint32_t byteswritten;                     /* File write/read counts */
	// uint8_t wtext[] = "This is STM32 working with FatFs\n"; /* File write buffer */
  pid_controller_current_Ia.limit = motor.voltage_limit;
  pid_controller_current_Id.limit = motor.voltage_limit;
  pid_controller_current_Iq.limit = motor.voltage_limit;
  pid_controller_current_OCP.limit = motor.voltage_limit;
  // uint8_t wlooptext[] = "This is STM32 working with FatFs in main loop\n"; /* File write buffer */
  // log_buf[0][0].LGSTATE = 0;
  for (size_t i = 0; i < 1024; i++)
  {
    float voltage = (float)(3300*i/1024);
    Mot_Conv[i] = (int16_t)10*(voltage/Mot_Curr/3.795-1000/3.795);
    Inv_Conv[i] = (int16_t)10*(float)((float)(2000*voltage-3300)/((float)(3.3-voltage)*3.8505));
    // Mot_Conv[i] = (int16_t)10*((float)(0.5*(3300*i/1024))/Mot_Curr/3.795-1000/3.795);
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
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED_D12_GPIO_Port,LED_D12_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_SD_GPIO_Port,LED_SD_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_TIM_GPIO_Port,LED_TIM_Pin,GPIO_PIN_RESET);

  //Get temperature sensor calibration data
  /* 0x1FF1E820 Calibration ADC value at 30 °C = 0x2fc0, 12224 */
  ts_cal1 = (float) *(uint16_t*) (TEMPSENSOR_CAL1_ADDR);
  /* 0x1FF1E840 Calibration ADC value at 110 °C = 0x3cb4, 15540 */
  ts_cal2 = (float) *(uint16_t*) (TEMPSENSOR_CAL2_ADDR);

  //Generate MCU conversion table
  for (size_t i = 0; i < 1024; i++)
  {
    MCU_Conv[i] = (int16_t) roundf(MCU_TemperatureCalculate(i<<6)*10);
  }

  // Calibrate ADC
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
  HAL_Delay(100);

  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim2);

  //Init SD files
  BSP_SD_Init();
  HAL_SD_InitCard(&hsd1);

  volatile int sdcard_status = HAL_SD_GetCardState(&hsd1);
  if(sdcard_status == HAL_SD_CARD_TRANSFER)
  {
    res = f_mount(&SDFatFS_RAM,(TCHAR const*)SDPath,1);
    // res = f_mount(&SDFatFS,"0",0);
    if (res != FR_OK)
    {
      Error_Handler();
    }
    else
    {
      #ifdef SDDEBUG
      res = f_open(&MyFile,TestFPath,FA_CREATE_ALWAYS | FA_WRITE);
      if (res != FR_OK)
      {
        Error_Handler();
      }
      else
      {
        set_one(log_test);
        f_write(&MyFile,&log_test,sizeof(log_test),(void *)&byteswritten);
        res = f_close(&MyFile);
        if(res!=FR_OK)
        {
          Error_Handler();
        }
      }
      #endif
    }
  }


  // Init ADC DMA
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_ADC1_arr,4);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)DMA_ADC2_arr,4);
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)DMA_ADC3_arr,6);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel0_dma1_stream2_tc_0,(uint32_t)tmp_DMA_ADC1_arr,(uint32_t)DMA_ADC1_arr,8,1);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel1_dma1_stream1_tc_0,(uint32_t)tmp_DMA_ADC2_arr,(uint32_t)DMA_ADC2_arr,8,1);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel2_dma1_stream4_tc_0,(uint32_t)tmp_DMA_ADC3_arr,(uint32_t)DMA_ADC3_arr,12,1);
  // HAL_ADC_Start_IT(&hadc3);

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

  //Wait for GATE READY Signal
  #ifdef WAIT_GATE_READY
  while (HAL_GPIO_ReadPin(GATE_Ready_GPIO_Port,GATE_Ready_Pin) != GPIO_PIN_SET)
  {
    HAL_Delay(1);
  }
  #endif

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

  UART_TX_Send(&huart1,"zero_electric_angle: %i \n",(int) floor(motor.zero_electric_angle/M_PI*180));

  Config_Fdcan1();

  // while (!got_date)
  // {
  //   HAL_Delay(10);
  // }

  //Get DateTime
  HAL_RTC_GetDate(&hrtc,&log_date,RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc,&log_time,RTC_FORMAT_BIN);

  snprintf(TextFPath, sizeof(TextFPath),FILENAME,(int)log_date.Year+2000,(int)log_date.Month,(int)log_date.Date,(int)log_time.Hours,(int)log_time.Minutes,(int)log_time.Seconds);
  // snprintf(TextFPath, sizeof(TextFPath),"text.bin");
  res = f_open(&MyFile,TextFPath,FA_CREATE_ALWAYS|FA_WRITE);
  if(res == FR_OK)
  {
    f_close(&MyFile);
  }
  f_open(&MyFile,TextFPath,FA_OPEN_APPEND|FA_WRITE);

  HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
  inverter_state = STATE_READY;
  error_state = ERROR_NONE;
  // INV_Statustypedef last_state = inverter_state;

  #ifdef TIMING
  prev_time = __HAL_TIM_GET_COUNTER(&htim5);
  #endif

  HAL_GPIO_WritePin(LED_SD_GPIO_Port,LED_SD_Pin,GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  prev_sd = __HAL_TIM_GET_COUNTER(&htim2);
  prev_new_file = __HAL_TIM_GET_COUNTER(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // continue;
    volatile int sd_now = __HAL_TIM_GET_COUNTER(&htim2);
    // uint32_t whileTest = sd_now;
    if (sd_now - prev_new_file >= 3000000)
    {
      f_close(&MyFile);
      snprintf(TextFPath, sizeof(TextFPath),FILENAME,(int)log_date.Year+2000,(int)log_date.Month,(int)log_date.Date,(int)log_time.Hours,(int)log_time.Minutes,(int)log_time.Seconds);
      // snprintf(TextFPath, sizeof(TextFPath),"text.bin");
      res = f_open(&MyFile,TextFPath,FA_CREATE_ALWAYS|FA_WRITE);
      if(res == FR_OK)
      {
        f_close(&MyFile);
      }
      f_open(&MyFile,TextFPath,FA_OPEN_APPEND|FA_WRITE);
      prev_new_file = sd_now;
    }    
    if (sd_now - prev_sd >= 1000)
    {
      HAL_GPIO_WritePin(LED_SD_GPIO_Port,LED_SD_Pin,GPIO_PIN_SET);

      if(last_got_date != got_date)
      {
        f_close(&MyFile);
        snprintf(TextFPath, sizeof(TextFPath),FILENAME,(int)log_date.Year+2000,(int)log_date.Month,(int)log_date.Date,(int)log_time.Hours,(int)log_time.Minutes,(int)log_time.Seconds);
        // snprintf(TextFPath, sizeof(TextFPath),"text.bin");
        res = f_open(&MyFile,TextFPath,FA_CREATE_ALWAYS|FA_WRITE);
        if(res == FR_OK)
        {
          f_close(&MyFile);
        }
        f_open(&MyFile,TextFPath,FA_OPEN_APPEND|FA_WRITE);
      }
            
      
      __disable_irq();
      uint8_t buf_num_to_sd = wr_log_buf_num;
      uint16_t index_to_sd = wr_log_index;
      wr_log_buf_num^=0x1;
      wr_log_index = 0;
      __enable_irq();

      if(max_sd_buf<index_to_sd)
      {
        max_sd_buf = index_to_sd;
      }

      res = f_write(&MyFile,log_buf[buf_num_to_sd],index_to_sd*sizeof(logger_t),(void *)&byteswritten);
      while (res != FR_OK)
      {
        if (res == FR_DISK_ERR)
        {
          f_open(&MyFile,TextFPath,FA_OPEN_APPEND|FA_WRITE);
        }
        else
        {
          break;
        }
        res = f_write(&MyFile,log_buf[buf_num_to_sd],index_to_sd*sizeof(logger_t),(void *)&byteswritten);
        if (res == FR_OK)
        {
          break;
        }
      }
      f_sync(&MyFile);

      last_got_date = got_date;

      #ifdef TIMING
      sd_td = __HAL_TIM_GET_COUNTER(&htim2) - sd_now;
      if (max_sdwrite < sd_td)
      {
        max_sdwrite = sd_td;
      }      
      #endif
      prev_sd = sd_now;
      

      HAL_GPIO_WritePin(LED_SD_GPIO_Port,LED_SD_Pin,GPIO_PIN_RESET);
    }
    if (sd_now - prev_sd < 0)
    {
      prev_sd = sd_now;
    }
    if (sd_now - prev_new_file >= 3000000)
    {
      prev_new_file = sd_now;
    }
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
// Read ADC values from DMA buffers
static void read_adc_buffers(uint16_t *adc1, uint16_t *adc2, uint16_t *adc3)
{
  SCB_InvalidateDCache_by_Addr(DMA_ADC1_arr, sizeof(DMA_ADC1_arr));
  for (size_t i = 0; i < 4; i++)
  {
    adc1[i] = DMA_ADC1_arr[i];
  }
  SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr, sizeof(DMA_ADC2_arr));
  for (size_t i = 0; i < 4; i++)
  {
    adc2[i] = DMA_ADC2_arr[i];
  }
  SCB_InvalidateDCache_by_Addr(DMA_ADC3_arr, sizeof(DMA_ADC3_arr));
  for (size_t i = 0; i < 6; i++)
  {
    adc3[i] = DMA_ADC3_arr[i];
  }
}

static void handle_state_and_ramp()
{
  if (inverter_state == STATE_RUNNING)
  {
    float delta = foc.percent_torque_requested - foc.last_percent;
    delta = _constrain(delta, -foc.max_ramp, foc.max_ramp);
    if (foc.last_percent != 0.0f && foc.percent_torque_requested == 0.0f)
    {
      if (foc.fast_stop_enable)
      {
        PID_integral_reset(&pid_controller_current_Iq);
        PID_integral_reset(&pid_controller_current_Id);
        for (size_t i = 0; i < 3; i++)
        {
          PID_integral_reset(&pid_controller_current_Iabc[i]);
        }
      }
      foc.last_percent = 0.0f;
    }
    foc.last_percent = foc.last_percent + delta;
    foc.last_percent = foc.percent_torque_requested;
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
    if(motor.voltage_power_supply < 50.0f)
    {
      Enter_READY_State();
    }
  }
  else
  {
    foc.last_percent = 0.0f;
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  }
  if(inverter_state == STATE_READY|| inverter_state == STATE_ERROR)
  {
    PID_reset(&pid_controller_current_Id);
    PID_reset(&pid_controller_current_Iq);
    PID_reset(&pid_controller_current_OCP);
    PID_reset(&pid_controller_current_Ia);
    for (size_t i = 0; i < 3; i++)
    {
      PID_reset(&pid_controller_current_Iabc[i]);
    }

    // pid_controller_current_Iq.output_prev = motor.dir*foc.filtered_RPM/motor.electrical_constant;
  }
}

static void update_encoder_and_voltage(uint16_t *adc2, uint16_t *adc3)
{
  float speed_rad, angle_pll;
  float raw_angle;  // throwaway buffer for Get_Encoder_Angle; only angle_pll is used
  Get_Encoder_Angle(adc2, &raw_angle, &speed_rad, &angle_pll);
  int8_t enc_err = 0;
  foc.angle_now = angle_pll;
  if (foc.angle_now != foc.angle_now)
  {
    enc_err = 1;
  }
  for (size_t i = 0; i < 4; i++)
  {
    if(adc2[i] < ENC_UV)
    {
      #ifndef OPEN_LOOP_SPEED
      Enter_ERROR_State(ERROR_ENC);
      #endif
    }
    if (adc2[i] > 4096-ENC_UV)
    {
      #ifndef OPEN_LOOP_SPEED
      Enter_ERROR_State(ERROR_ENC);
      #endif
    }
  }
  enc_sum -= enc_buf[enc_index];
  enc_buf[enc_index] = enc_err;
  enc_sum += enc_buf[enc_index];
  if (enc_sum > ENC_TIME/2)
  {
    if (inverter_state == STATE_RUNNING)
    {
      #ifndef OPEN_LOOP_SPEED
      Enter_ERROR_State(ERROR_ENC);
      #endif
    }
  }

  foc.angle_now = _normalizeAngle(foc.angle_now);

  motor.voltage_power_supply = (float)adc3[0] * DCVPLSB;
  motor.voltage_limit = motor.voltage_power_supply;

  float angular_vel = 0.0f;
  cal_angular_vel(foc.angle_now, &angular_vel);
  foc.filtered_RPM = LowPassFilter_operator((float)motor.dir*angular_vel/4/2/M_PI*60, &filter_RPM);
}

static void measure_currents_and_check_oc(uint16_t *adc1, float *phase_dc)
{
  int8_t soft_oc_detected = 0;
  soft_oc_sum -= soft_oc_buf[soft_oc_index];

  for (int i = 0; i < 3; i++) {
      foc.current_phase[i] = (float)(adc1[i] - foc.current_offset[i]) * ACAPLSB;
      // OCP
      // filtered_Iabc[i] = LowPassFilter_operator(foc.current_phase[i],&filter_current_Iabc[i]);
      phase_dc[i] = LowPassFilter_operator(foc.current_phase[i], &filter_crrent_DC_Iabc[i]);
      if (foc.current_phase[i] > ACAOCP||foc.current_phase[i] < -ACAOCP)
      {
        soft_oc_detected = 1;
      }
      // foc.current_phase[i] = filtered_Iabc[i];
  }

  soft_oc_buf[soft_oc_index] = soft_oc_detected;
  soft_oc_sum += soft_oc_buf[soft_oc_index];
  if (soft_oc_sum > SOFT_OC_TIME/2 && inverter_state == STATE_RUNNING)
  {
    Enter_ERROR_State(ERROR_INSTANT_OC);
  }
  soft_oc_index++;
  if (soft_oc_index == SOFT_OC_TIME)
  {
    soft_oc_index = 0;
  }

  pid_controller_current_Ia.limit = motor.voltage_limit;
  pid_controller_current_Id.limit = motor.voltage_limit;
  pid_controller_current_Iq.limit = motor.voltage_limit;
  pid_controller_current_OCP.limit = motor.voltage_limit;
  pid_controller_current_Iabc[0].limit = motor.voltage_limit;
  pid_controller_current_Iabc[1].limit = motor.voltage_limit;
  pid_controller_current_Iabc[2].limit = motor.voltage_limit;
}

static void foc_control_step(float *phase_dc, float *Iabc_controller_output)
{
  float temp_derate = 1.0f;
  #ifdef DISALBE_MOT_OT
  temp_derate = _constrain(((float)abs(T_Mot)-(float)T_DERATE_END)/(T_DERATE_START-T_DERATE_END),0.0f,1.0f);
  #endif

  foc.last_percent = _constrain(foc.last_percent, -temp_derate, temp_derate);
  float target_Is = motor.max_current * foc.last_percent;
  foc.target_Iq = target_Is;
  foc.target_Id = 0.0f;

  float Id, Iq;
  cal_Idq(foc.current_phase, _electricalAngle(foc.angle_now, motor.pole_pairs), &Id, &Iq);
  foc.filtered_Iq = LowPassFilter_operator(Iq, &filter_current_Iq);
  foc.filtered_Id = LowPassFilter_operator(Id, &filter_current_Id);

  float Vd_decoupling = (-1.0f)*(4*foc.filtered_RPM*2*M_PI/60)*motor.Lq*foc.filtered_Iq;
  float Vq_decoupling = (4*foc.filtered_RPM*2*M_PI/60)*(motor.Ld*foc.filtered_Id+motor.flux_linkage_m);
  Vq_decoupling = _constrain(Vq_decoupling, -motor.voltage_limit, motor.voltage_limit);
  Vd_decoupling = _constrain(Vd_decoupling, -motor.voltage_limit, motor.voltage_limit);

  foc.Id_fw = 0.0f;
  foc.Iq_fw = foc.target_Iq;
  foc.Id_MTPA = 0.0f;

  #ifdef FIELD_WEAKENING
  foc.Id_fw = field_weaking_control(fabsf(foc.filtered_RPM), fabsf(foc.filtered_Iq), fabsf(foc.Iq_controller_output), motor.voltage_limit);
  #endif

  #ifdef MTPA
  foc.Id_MTPA = MTPA_control(fabsf(filtered_Iq));
  #endif

  #ifdef FIELD_WEAKENING_ANGLE
  foc.Id_fw = foc.Id_MTPA;
  float fw_angle = field_weaking_angle_control(&foc.Iq_fw, &foc.Id_fw, foc.Iq_controller_output, foc.Id_controller_output, motor.voltage_limit);
  #endif

  #ifdef PERMANENT_FLUX
  foc.Id_fw = -MAX_FLUX_ID;
  #endif

  float Id_flux_control = foc.Id_fw < foc.Id_MTPA ? foc.Id_fw : foc.Id_MTPA;
  // foc.target_Id = _constrain(Id_flux_control,(-MAX_FLUX_ID)*fabsf(foc.last_percent)*4.0f,0.0f);
  foc.target_Id = Id_flux_control;
  // if(filtered_RPM > motor.voltage_power_supply/(motor.electrical_constant+0.02f) && foc.last_percent == 0.0f)
  // {
  //   foc.target_Iq = -1.0f * _sign(filtered_RPM);
  // }
  // foc.target_Id = Id_flux_control;
  float max_Iq = sqrtf(motor.max_current*motor.max_current - foc.target_Id*foc.target_Id);
  foc.target_Iq = _constrain(foc.target_Iq, -max_Iq,max_Iq);
  #ifdef OVERSPEED_PROT
  foc.target_Iq = foc.Id_fw<-MAX_TORQUE_FW_ID?0.0f:foc.target_Iq;
  #endif

  foc.Iq_controller_output = PID_operator(foc.target_Iq - foc.filtered_Iq, &pid_controller_current_Iq);
  foc.Id_controller_output = PID_operator(foc.target_Id - foc.filtered_Id, &pid_controller_current_Id);

  #ifdef Decouopling
  // Decoupling
  foc.Id_controller_output += Vd_decoupling;
  foc.Iq_controller_output += Vq_decoupling;
  #endif

  foc.Id_controller_output = _constrain(foc.Id_controller_output, -motor.voltage_limit, motor.voltage_limit);
  foc.Iq_controller_output = _constrain(foc.Iq_controller_output, -motor.voltage_limit, motor.voltage_limit);
  // float max_Id = sqrtf(motor.voltage_limit*motor.voltage_limit - foc.Iq_controller_output*foc.Iq_controller_output);
  // foc.Id_controller_output = _constrain(foc.Id_controller_output,-max_Id,max_Id);

  for (size_t i = 0; i < 3; i++)
  {
    Iabc_controller_output[i] = PID_operator(-phase_dc[i], &pid_controller_current_Iabc[i]);
  }
  if (abs(foc.filtered_RPM) > 1000 && foc.enable_dc_control == 0)
  {
    foc.enable_dc_control = 1;
  }
  else if (abs(foc.filtered_RPM) < 750 && foc.enable_dc_control == 1)
  {
    foc.enable_dc_control = 0;
  }
  if (!foc.enable_dc_control)
  {
    for (size_t i = 0; i < 3; i++)
    {
      Iabc_controller_output[i] = 0.0f;
      PID_reset(&pid_controller_current_Iabc[i]);
    }
  }

  setPhaseVoltage(foc.Iq_controller_output, foc.Id_controller_output, _electricalAngle(foc.angle_now, motor.pole_pairs),TIM1,-Iabc_controller_output[0],-Iabc_controller_output[1],-Iabc_controller_output[2]);
}

static void update_leds()
{
  if (indexLED == foc.freq/2)
  {
    if (inverter_state == STATE_READY)
    {
      HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }
    else if (inverter_state == STATE_ERROR)
    {
      HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
    }

    indexLED = 0;
  }
}

static void periodic_can_report(uint16_t *adc1, uint16_t *adc3)
{
  telem.report_status = 0;
  telem.report_DCV = (uint16_t) roundf(motor.voltage_power_supply*100);
  telem.report_DCA = (int16_t) roundf((float)(adc1[3] - foc.current_offset[3])*DCAPLSB*100);
  if (indexHeartbeat == foc.freq/10)
  {
    CAN_Send_Temp(adc3);
    CAN_Send_State(telem.report_DCV, telem.report_DCA);
    CAN_Send_Heartbeat();
    CAN_Send_Perameter();
    // CAN_Send_Heartbeat();
    indexHeartbeat = 0;
  }

  if (inverter_state == STATE_READY)
  {
    telem.report_status |= REPORT_STATUS_READY;
  } else if (inverter_state == STATE_RUNNING)
  {
    telem.report_status |= REPORT_STATUS_ENABLED;
  } else if (inverter_state == STATE_ERROR)
  {
    telem.report_status |= REPORT_STATUS_FAULT;
  }
  if (adc3[0]*DCVPLSB > 55)
  {
    telem.report_status |= REPORT_STATUS_HV;
  }

  if (indexStatus == foc.freq/100)
  {
    // int_RPM = (int) lroundf(filtered_RPM);
    int16_t report_RPM = (int16_t) roundf(foc.filtered_RPM);
    // int16_t report_RPM = (int16_t) roundf(zero_cross/4*100*60);
    foc.zero_cross = 0.0f;
    int16_t report_torque = (int16_t) roundf(foc.filtered_Iq/motor.max_current*1000);
    // int16_t report_torque = (int16_t) roundf(foc.Ia/motor.max_current*1000);
    // report_torque = LowPassFilter_operator(report_torque,&filter_report_torque);
    CAN_Send_Status(telem.report_status, report_torque, report_RPM);
    indexStatus = 0;
  }
}

static void fill_log_entry(uint16_t *adc2, float *current_phase_dc, float *Iabc_controller_output)
{
  telem.IU_100 = (int16_t)roundf(foc.current_phase[0]*100);
  telem.IV_100 = (int16_t)roundf(foc.current_phase[1]*100);
  telem.IW_100 = (int16_t)roundf(foc.current_phase[2]*100);
  HAL_RTC_GetDate(&hrtc, &log_date, RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc, &log_time, RTC_FORMAT_BIN);
  if (log_time.Seconds != last_sec)
  {
    log_subsec = 0;
    last_sec = log_time.Seconds;
  }
  logger_t *entry = &log_buf[wr_log_buf_num][wr_log_index%3600];
  entry->LGHR = log_time.Hours;
  entry->LGMIN = log_time.Minutes;
  entry->LGSEC = log_time.Seconds;
  entry->LGERR = error_state;
  entry->LGSUBSEC = log_subsec;
  entry->LGDCV = telem.report_DCV;
  entry->LGDCA = telem.report_DCA;
  entry->LGIU = telem.IU_100;
  entry->LGIV = telem.IV_100;
  entry->LGIW = telem.IW_100;
  entry->LGVQ = (int16_t) roundf(foc.Iq_controller_output*10);
  entry->LGVD = (int16_t) roundf(foc.Id_controller_output*10);
  entry->LGSINE = adc2[0] - adc2[1];
  entry->LGCOS = adc2[2] - adc2[3];
  entry->LGANG = (uint16_t) roundf(foc.angle_now*100*180/M_PI);
  entry->LGTCMD = (int16_t) roundf(foc.last_percent*1000);
  entry->LGSTATE = telem.report_status;
  entry->LGVU = TIM1->CCR1;
  entry->LGVV = TIM1->CCR2;
  entry->LGVW = TIM1->CCR3;
  entry->LGRPM = (int16_t) roundf(foc.filtered_RPM);
  entry->LGID = (int16_t) roundf(foc.filtered_Id*100);
  entry->LGIQ = (int16_t) roundf(foc.filtered_Iq*10);
  entry->LGZERO = (uint16_t) soft_oc_sum;
  entry->LGDCIU = (int16_t) roundf(foc.target_Id*100);
  entry->LGDCIV = (int16_t) roundf(foc.target_Iq*100);
  entry->LGDCIW = (int16_t) roundf(current_phase_dc[2]*100);
  entry->LGVA = (int16_t) roundf(foc.Id_fw*10);
  entry->LGVB = (int16_t) roundf(foc.Id_MTPA*10);
  entry->LGVC = (int16_t) roundf(Iabc_controller_output[2]*10);
  entry->LGRMSIU = (uint16_t) __HAL_TIM_GET_COUNTER(&htim5)>>16;
  entry->LGRMSIV = (uint16_t) __HAL_TIM_GET_COUNTER(&htim5);
  entry->LGRMSIW = (uint16_t) roundf(RMS_sum[2]/100);
  entry->LGLOGBUF = wr_log_index;

  log_subsec++;
  wr_log_index++;
}

static void check_hw_overcurrent()
{
  if(inverter_state == STATE_RUNNING)
  {
    oc_sum -= oc_buf[oc_index];
    oc_buf[oc_index] = (HAL_GPIO_ReadPin(OC_Fault_GPIO_Port,OC_Fault_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    oc_sum += oc_buf[oc_index];
    if (oc_sum > HW_OC_TIME/2)
    {
      Enter_ERROR_State(ERROR_HW_OC);
    }
  }
  oc_index++;
  if (oc_index == HW_OC_TIME)
  {
    oc_index = 0;
  }
}

#ifdef RMSOCP
static void check_rms_overcurrent()
{
  // Moving RMS for phase currents
  for (size_t i = 0; i < 3; i++)
  {
    RMS_sum[i]-=(RMS_buf[i][indexRMS]*RMS_buf[i][indexRMS]);
  }
  RMS_buf[0][indexRMS] = IU_100;
  RMS_buf[1][indexRMS] = IV_100;
  RMS_buf[2][indexRMS] = IW_100;
  for (size_t i = 0; i < 3; i++)
  {
    RMS_sum[i]+=(RMS_buf[i][indexRMS]*RMS_buf[i][indexRMS]);
  }
  indexRMS++;
  if(indexRMS==10000)
  {
    indexRMS = 0;
  }
  for (size_t i = 0; i < 3; i++)
  {
    if(RMS_sum[i]/10000 > MOVRMSOCP)
    {
      Enter_ERROR_State(ERROR_RMS_OC);
    }
  }
}
#endif

#ifdef CAN_OT_FAULT
static void check_can_timeout()
{
  // CAN fault detect
  if (CAN_Timer == foc.freq && inverter_state == STATE_RUNNING)
  {
    foc.enable_hw_oc = 0;
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
    inverter_state = STATE_READY;
  }
}
#endif

#ifdef TIMING
static void update_timing_stats(uint32_t tick_start)
{
  loop_time = __HAL_TIM_GET_COUNTER(&htim5) - tick_start;
  if (loop_time > max_time)
  {
    max_time = loop_time;
  }
  if (loop_time < min_time)
  {
    min_time = loop_time;
  }
  int btw_time = tick_start - prev_time;
  if (btw_time > max_btw)
  {
    max_btw = btw_time;
  }
  prev_time = tick_start;
  if (indexTimer == foc.freq*10)
  {
    max_time = 0;
    min_time = INT16_MAX;
    max_btw = 0;
    indexTimer = 0;
    max_sdwrite = 0;
  }
  indexTimer++;
}
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim != &htim1) return;

  if (!foc.run)
  {
    foc.run = 1;
    return;
  }
  foc.run = 0;

  HAL_GPIO_TogglePin(LED_TIM_GPIO_Port, LED_TIM_Pin);

  // ADC
  uint16_t ADC1_arr[4] = {0};
  uint16_t ADC2_arr[4] = {0};
  uint16_t ADC3_arr[6] = {0};
  read_adc_buffers(ADC1_arr, ADC2_arr, ADC3_arr);

  handle_state_and_ramp();

  #ifdef TIMING
  uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim5);
  #endif

  indexLED++;
  indexHeartbeat++;
  indexStatus++;
  CAN_Timer++;

  update_encoder_and_voltage(ADC2_arr, ADC3_arr);

  float current_phase_dc[3] = {0.0f};
  measure_currents_and_check_oc(ADC1_arr, current_phase_dc);

  float Iabc_controller_output[3] = {0.0f};
  foc_control_step(current_phase_dc, Iabc_controller_output);

  update_leds();

  periodic_can_report(ADC1_arr, ADC3_arr);

  // Logging
  fill_log_entry(ADC2_arr, current_phase_dc, Iabc_controller_output);

  check_hw_overcurrent();

  #ifdef RMSOCP
  check_rms_overcurrent();
  #endif

  #ifdef CAN_OT_FAULT
  check_can_timeout();
  #endif

  #ifdef TIMING
  update_timing_stats(tick_start);
  #endif

  HAL_GPIO_TogglePin(LED_TIM_GPIO_Port,LED_TIM_Pin);
}

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
