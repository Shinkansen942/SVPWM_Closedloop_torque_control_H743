/* USER CODE BEGIN Header */
/*
  TODO:
  UART Control
  Change ADC to Phy conversion ratio -> use 1~4k array to store data
  SDMMC

*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "fdcan.h"
#include "i2c.h"
#include "mdma.h"
#include "memorymap.h"
#include "sdmmc.h"
#include "spi.h"
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
#define CAL_ZERO_ANGLE 1
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
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
float MCU_MapValue(uint16_t in_value, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
float MCU_TemperatureCalculate(uint16_t ts_data);
void Enter_ERROR_State(void);
void config_fdcan1(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char data[500];


float open_loop_timestamp=0;
float zero_electric_angle=0;
float shaft_angle=0;
float voltage_limit=24;
float voltage_power_supply=24;
int period=2596; // period for the PWM
int dir=1; // anti clockwise direction is 1 , clockwise is -1
int pole_pairs=1;
int indexLED=0;
int indexHeartbeat=0;
int indexStatus=0;
uint16_t raw1,raw2,raw3;
float motor_target= M_PI/6;
float Ts=(float)1/10000;
float Kp=0.167f;
float angle_prev=-1.0f;
float target_vel=5;
const float torque_constant = 0.492f; //Nm/A
const float max_torque = 25;
float percent_torque_requested = 0.04f;
uint16_t ADC_VAL[3];
uint16_t current_offset[3];
double current_phase[3];
const int adc_range=4095;
const double Vref=3.3;    // Voltage
const double Rsense=0.33; // Ohm
const float MotCPLSB = 10.0f;
const float InvCPLSB = 10.0f;
const float DCVPLSB = 10.0f;
const float DCAPLSB = 10.0f;
const float ACAPLSB = 10.0f;
int max_time = 0;

INV_Statustypedef inverter_state = STATE_INIT;
__attribute__((section("._ADC1_Area"))) uint16_t ADC1_arr[4] = {0};
__attribute__((section("._ADC2_Area"))) uint16_t ADC2_arr[4] = {0};
__attribute__((section("._ADC3_Area"))) uint16_t ADC3_arr[6] = {0};
// uint16_t ADC1_arr[4] = {0};
// uint16_t ADC2_arr[4] = {0};
// uint16_t ADC3_arr[6] = {0};

FDCAN_TxHeaderTypeDef HeartBeatHeader = { .Identifier = CAN_ID_HEARTBEAT+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_1,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x01};
FDCAN_TxHeaderTypeDef TempHeader      = { .Identifier = CAN_ID_TEMPERATURE+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_2,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x02};
FDCAN_TxHeaderTypeDef StateHeader     = { .Identifier = CAN_ID_STATE+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_8,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x03};
FDCAN_TxHeaderTypeDef StatusHeader    = { .Identifier = CAN_ID_STATUS+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_4,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x04};
       
FDCAN_RxHeaderTypeDef RxHeader1;

int isSent = 1;
uint16_t control;
int CAN_Timer = 0;
//const double KV= 2375.0/12.0; //KV number (RPM is 2149 - 2375, when operating voltage is 12V)

struct LowPassFilter filter= {.Tf=0.01,.y_prev=0.0f}; //Tf=10ms
struct LowPassFilter filter_current= {.Tf=0.05,.y_prev=0.0f}; //Tf=50ms
// limit=voltage_power_supply/2;
struct PIDController pid_controller = {.P=0.5,.I=0.1,.D=0.0,.output_ramp=100.0,.limit=6,.error_prev=0,.output_prev=0,.integral_prev=0};

struct PIDController pid_controller_current = {.P=1.0,.I=0.1,.D=0.0,.output_ramp=100.0,.limit=6,.error_prev=0,.output_prev=0,.integral_prev=0};

// SD_HandleTypeDef hsd;
FIL TestFile;
static uint8_t buffer[_MAX_SS]; /* a work buffer for the f_mkfs() */

void UART_TX_Send(UART_HandleTypeDef *huart,const char *format,...)
{
  __attribute__((section("._RAM_Area"))) static char tdata[500];
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

void CAN1_SetMsg(FDCAN_TxHeaderTypeDef *pTxHeader, const uint8_t *pTxData)
{
    HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, pTxHeader, pTxData, FDCAN_TX_BUFFER0);
    /* 启动FDCAN模块 */
    HAL_FDCAN_Start(&hfdcan1);
    /* 发送缓冲区消息 */
    HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  volatile FRESULT res = 1000;                                 /* FatFs function common result code */
	uint32_t byteswritten, bytesread;                     /* File write/read counts */
	uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
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
  MX_FATFS_Init();
  MX_ADC2_Init();
  MX_SDMMC1_SD_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_T_GPIO_Port,LED_T_Pin,GPIO_PIN_SET);

  // Calibrate ADC
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
  HAL_Delay(100);

  //Init SD files
  volatile HAL_StatusTypeDef res_hal;
  res_hal = HAL_SD_Init(&hsd1);
  if (res_hal != HAL_OK)
  {
    Error_Handler();
  }
  res_hal = HAL_SD_InitCard(&hsd1);
  if (res_hal != HAL_OK)
  {
    Error_Handler();
  }
  // while(1)
  // {
  //   int sdcard_status = HAL_SD_GetCardState(&hsd1);
  //   HAL_GPIO_TogglePin(LED_T_GPIO_Port,LED_T_Pin);
  //   HAL_Delay(100);
  // }
  volatile int sdcard_status = HAL_SD_GetCardState(&hsd1);
  if(sdcard_status == HAL_SD_CARD_TRANSFER)
  {
    res = f_mount(&SDFatFS,(TCHAR const*)SDPath,1);
    // res = f_mount(&SDFatFS,"0",0);
    if (res != FR_OK)
    {
      Error_Handler();
    }
    else
    {
      res = f_open(&TestFile,"0:/Test.txt",FA_CREATE_ALWAYS | FA_WRITE);
      if (res != FR_OK)
      {
        Error_Handler();
      }
      else
      {
        f_write(&TestFile,wtext,sizeof(wtext),(void *)&byteswritten);
        if(f_close(&TestFile)!=FR_OK)
        {
          Error_Handler();
        }
      }
    }
  }

  // Init ADC DMA
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_arr,4);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_arr,4);
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)ADC3_arr,6);
  // HAL_ADC_Start_IT(&hadc3);

  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start(&htim5);
  calibrateOffsets(current_offset,ADC1_arr);
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  if(CAL_ZERO_ANGLE)
  {
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
    setPhaseVoltage(3,0,_electricalAngle(M_PI*1.5f,pole_pairs),TIM1);
    HAL_Delay(3000);
    // uint16_t read_raw=read(&hspi1, SPI1_CSn_GPIO_Port,SPI1_CSn_Pin,AS5048A_ANGLE);
    float raw_angle;
    Get_Encoder_Angle(ADC2_arr,&raw_angle);
    zero_electric_angle=_electricalAngle(raw_angle,pole_pairs);
    setPhaseVoltage(0,0,_electricalAngle(M_PI*1.5f,pole_pairs),TIM1);
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  }  
  inverter_state = STATE_READY;
  // TODO condense into single function
  // sprintf(data, "zero_electric_angle: %i \n", (int) floor(zero_electric_angle/M_PI*180));
  // if(isSent)
  // {
  //   HAL_UART_Transmit_DMA(&huart1,data,sizeof(data));
  //   isSent = 0;
  // }
  UART_TX_Send(&huart1,"zero_electric_angle: %i \n",(int) floor(zero_electric_angle/M_PI*180));
  // TODO 
  // CDC_Transmit_FS((uint8_t*) data, strlen(data));
  HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
  config_fdcan1();
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 24;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 3;
  PeriphClkInitStruct.PLL2.PLL2N = 300;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 20;
  PeriphClkInitStruct.PLL2.PLL2R = 4;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//   if(hadc->Instance == hadc3.Instance) {
//     volatile uint32_t adc_val = HAL_ADC_GetValue(hadc);
//     volatile int a = 0;
//   }
// }


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim2 )
  {
    
    uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim5);

    

    indexLED++;
    indexHeartbeat++;
    indexStatus++;
    CAN_Timer++;

    float angle_now;
    Get_Encoder_Angle(ADC2_arr,&angle_now);

    float angular_vel=cal_angular_vel(angle_now);
    float filtered_vel=LowPassFilter_operator(angular_vel,&filter);
    float target_torque = max_torque*percent_torque_requested;
    float target_Iq = target_torque/torque_constant;
    for (int i=0;i<3;i++){
       	current_phase[i] =(float)  (ADC1_arr[i]-current_offset[i])/ACAPLSB;
        if (current_phase[i] > ACAOCP||current_phase[i] < -ACAOCP)
        {
          Enter_ERROR_State();
        }
    }

    float Iq=cal_Iq(current_phase, _electricalAngle(angle_now, pole_pairs));
    float filtered_Iq=LowPassFilter_operator(Iq,&filter_current);
    float Id=cal_Id(current_phase, _electricalAngle(angle_now,pole_pairs));
    float filtered_Id=LowPassFilter_operator(Id,&filter_current);

    float current_controller_output=PID_operator(target_Iq-filtered_Iq,&pid_controller_current);
    float Id_controller_output=PID_operator(0-filtered_Id,&pid_controller_current);

    setPhaseVoltage(_constrain(current_controller_output,-voltage_power_supply/2,voltage_power_supply/2),  _constrain(Id_controller_output,-voltage_power_supply/2,voltage_power_supply/2), _electricalAngle(angle_now, pole_pairs),TIM8);
//    sprintf(data, "angle_now : %i \t angle_prev : %i \n", (int) floor(angle_now/M_PI*180), (int) floor(angle_prev/M_PI*180));
//    CDC_Transmit_FS((uint8_t*) data, strlen(data));
    // sprintf(data, "angular_vel : %i \n", (int) floor(angular_vel/M_PI*180));
    // sprintf(data, "filtered_vel : %i \n", (int) floor(filtered_vel/M_PI*180)); 
    // sprintf(data, "target_torque : %i \n", (int) floor(target_torque));
    // sprintf(data, "real_torque : %i \n", (int) floor(filtered_Iq*torque_constant));   
    // CDC_Transmit_FS((uint8_t*) data, strlen(data));
//    sprintf(data, "open loop control \n");
//    sprintf(data, "angle: %u \n", read_raw);
//    CDC_Transmit_FS((uint8_t*) data, strlen(data));
//    sprintf(data, "angle_now : %i \n", (int) floor(angle_now*180/M_PI));
//        CDC_Transmit_FS((uint8_t*) data, strlen(data));

    // FDCAN_ProtocolStatusTypeDef protocolStatus = {};
    // HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocolStatus);
    // if (protocolStatus.BusOff) {
    //     CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
    // }

    if (indexLED == 500)
    {
    	HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    	HAL_GPIO_TogglePin(LED_T_GPIO_Port, LED_T_Pin);
      UART_TX_Send(&huart1,"ping");
    	indexLED=0;
    }
    if (indexHeartbeat == 1000)
    {
      uint8_t HBData = 0x7f;
      CAN1_SetMsg(&HeartBeatHeader,&HBData);
      uint8_t TempData[2];
      float T_Mot = (float) ADC3_arr[2]*MotCPLSB;
      TempData[1] = (uint8_t) round(T_Mot/0.5);
      // volatile float T_MCU = (float)MCU_TemperatureCalculate(ADC3_arr[1]<<4);
      volatile float T_MCU = (float)MCU_TemperatureCalculate(ADC3_arr[1])-50;
      float T_U = (float)ADC3_arr[3]*InvCPLSB;
      float T_V = (float)ADC3_arr[4]*InvCPLSB;
      float T_W = (float)ADC3_arr[5]*InvCPLSB;
      float T_Report = fmax(fmax(T_MCU,T_U),fmax(T_V,T_W));
      TempData[0] = (uint8_t) round(T_Report/0.5);
      CAN1_SetMsg(&TempHeader,TempData);
      indexHeartbeat = 0;
    }
    if (indexStatus == 100)
    {
      float Ia = sqrt(filtered_Id*filtered_Id+filtered_Iq*filtered_Iq);
      int16_t report_RPM = (int16_t) roundf(filtered_vel/2/M_PI*60);
      int16_t report_torque = (int16_t) roundf(Ia*torque_constant/max_torque*1000);
      uint16_t report_DCV = (uint16_t) roundf(ADC3_arr[0]*DCVPLSB*100);
      int16_t report_DCA = (int16_t) roundf(ADC1_arr[3]*DCAPLSB*100);
      uint8_t StateData[8];
      StateData[0] = report_RPM;
      StateData[1] = report_RPM >> 8;
      StateData[2] = report_torque;
      StateData[3] = report_torque >> 8;
      StateData[4] = report_DCV;
      StateData[5] = report_DCV >> 8;
      StateData[6] = report_DCA;
      StateData[7] = report_DCA >> 8;
      CAN1_SetMsg(&StateHeader,StateData);
      uint16_t report_status = 0;
      if(inverter_state == STATE_READY) 
      {
        report_status |= REPORT_STATUS_READY;
      }else if(inverter_state == STATE_RUNNING)
      {
        report_status |= REPORT_STATUS_ENABLED;
      }else if(inverter_state == STATE_ERROR)
      {
        report_status |= REPORT_STATUS_FAULT;
      }
      if(ADC3_arr[0]*DCVPLSB > 60)
      {
        report_status |= REPORT_STATUS_HV;
      }
      uint8_t StatusData[4];
      StatusData[0] = report_status;
      StatusData[1] = report_status>>8;
      StatusData[2] = ((int16_t)percent_torque_requested*10);
      StatusData[3] = ((int16_t)percent_torque_requested*10)>>8;
      CAN1_SetMsg(&StatusHeader,StatusData);
      indexStatus = 0;
    }
    // fault detect
    if(CAN_Timer == 10000 && inverter_state == STATE_RUNNING)
    {
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
      inverter_state = STATE_READY;
    }
    uint32_t loop_time = __HAL_TIM_GET_COUNTER(&htim5)-tick_start;
    if (loop_time > max_time)
    {
      max_time = loop_time;
    }
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	isSent = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if(hadc == &hadc1)
  {
    SCB_InvalidateDCache_by_Addr(ADC1_arr,sizeof(ADC1_arr));
  }
  if(hadc == &hadc2)
  {
    SCB_InvalidateDCache_by_Addr(ADC2_arr,sizeof(ADC2_arr));
  }
  if(hadc == &hadc2)
  {
    SCB_InvalidateDCache_by_Addr(ADC3_arr,sizeof(ADC3_arr));
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=0)
  {
    uint8_t RxData1[4];
    HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&RxHeader1,RxData1);
    if(hfdcan->Instance == FDCAN1)
    {
      int16_t torque_command;
      control = RxData1[0] | (uint16_t)RxData1[1] << 8;
      // enable
      if(control & CTRL_ENABLE && inverter_state == STATE_READY)
      {
        inverter_state = STATE_RUNNING;
        percent_torque_requested = 0;
        HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
      // disable
      }else if(!(control & CTRL_ENABLE) && inverter_state == STATE_RUNNING) 
      {
        inverter_state = STATE_READY;
        percent_torque_requested = 0;
        HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
      }
      // fault reset
      if(CTRL_FAULT_RESET && !(control & CTRL_FAULT_RESET) && inverter_state == STATE_ERROR) 
      {
        inverter_state = STATE_READY;
      }
      torque_command = data[2] | data[3] << 8;
      percent_torque_requested = (float)torque_command/1000;
      CAN_Timer = 0;
    }
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if(hfdcan->Instance == FDCAN1)
  {
    MX_FDCAN1_Init();
  }
}

void config_fdcan1(void)
{
  FDCAN_FilterTypeDef CAN1RxFilterConfig;
  CAN1RxFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN1RxFilterConfig.FilterIndex = 0;
  CAN1RxFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  CAN1RxFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  CAN1RxFilterConfig.FilterID1 = CAN_ID_CONTROL+MOT_ID;
  CAN1RxFilterConfig.FilterID2 = 0x7FF;
  CAN1RxFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1,&CAN1RxFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,ENABLE,ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_BUS_OFF,0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_FDCAN_Start(&hfdcan1);
}

void Enter_ERROR_State(void)
{
  inverter_state = STATE_ERROR;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
}

float MCU_MapValue(uint16_t in_value, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (float)(in_value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}
 
float MCU_TemperatureCalculate(uint16_t ts_data)
{
  /* 0x1FF1E820 Calibration ADC value at 30 °C = 0x2fc0, 12224 */
  uint16_t ts_cal1 = *(uint16_t*) (TEMPSENSOR_CAL1_ADDR);
  /* 0x1FF1E840 Calibration ADC value at 110 °C = 0x3cb4, 15540 */
  uint16_t ts_cal2 = *(uint16_t*) (TEMPSENSOR_CAL2_ADDR);
 
//  return (80 * (ts_data - ts_cal1)) / (ts_cal2 - ts_cal1) + 30;
 
  return MCU_MapValue(ts_data, ts_cal1, ts_cal2, TEMPSENSOR_CAL1_TEMP, TEMPSENSOR_CAL2_TEMP);
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

#ifdef  USE_FULL_ASSERT
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
