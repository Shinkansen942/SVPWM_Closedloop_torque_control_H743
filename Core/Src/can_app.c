/*
 * can_app.c
 *
 * CAN Application Layer
 */

#include "can_app.h"
#include "canid.h"
#include "config.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "rtc.h"
#include "state_machine.h"
#include <string.h>
#include <math.h>
#include "time.h"
#include "foc_loop.h"

/* ================================================================
 *  Extern declarations – variables defined in other modules
 * ================================================================ */

// --- Motor parameters (from main.c) ---
extern motor_params_t motor;

// --- Motor / FOC control (from main.c) ---
extern foc_state_t foc;
extern float    Mot_Curr;
extern uint32_t indexMusic;

// --- OC / Encoder fault buffers (from main.c) ---
extern uint8_t  oc_buf[];
extern uint16_t oc_index;
extern uint16_t oc_sum;
extern uint8_t  soft_oc_buf[];
extern uint16_t soft_oc_index;
extern uint16_t soft_oc_sum;
extern uint8_t  enc_buf[];
extern uint16_t enc_index;
extern uint16_t enc_sum;

// --- Low-pass filters (from main.c) ---
extern lpf_t filter_Idfw;

// --- PID controllers (from main.c) ---
extern pidc_t pid_controller_current_Iq;
extern pidc_t pid_controller_current_Id;
extern pidc_t pid_controller_current_Iabc[3];

// --- Temperature variables & lookup tables (from main.c) ---
extern telemetry_t telem;
extern int16_t       Mot_Conv[];
extern int16_t       MCU_Conv[];
extern const int16_t Inv_Conv[];

// --- RTC helpers ---
extern uint8_t got_date;

/* ================================================================
 *  Module-private variables
 * ================================================================ */

// TX headers (only used inside this file)
static FDCAN_TxHeaderTypeDef HeartBeatHeader = {
    .Identifier = CAN_ID_HEARTBEAT + MOT_ID, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_1, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_STORE_TX_EVENTS, .MessageMarker = 0x01};

static FDCAN_TxHeaderTypeDef TempHeader = {
    .Identifier = CAN_ID_TEMPERATURE + MOT_ID, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_6, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_STORE_TX_EVENTS, .MessageMarker = 0x02};

static FDCAN_TxHeaderTypeDef StateHeader = {
    .Identifier = CAN_ID_STATE + MOT_ID, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_4, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_STORE_TX_EVENTS, .MessageMarker = 0x03};

static FDCAN_TxHeaderTypeDef StatusHeader = {
    .Identifier = CAN_ID_STATUS + MOT_ID, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_6, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_STORE_TX_EVENTS, .MessageMarker = 0x04};

static FDCAN_TxHeaderTypeDef PerameterHeader = {
    .Identifier = CAN_ID_PERAM + MOT_ID, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
    .DataLength = FDCAN_DLC_BYTES_5, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
    .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_STORE_TX_EVENTS, .MessageMarker = 0x05};

static FDCAN_RxHeaderTypeDef RxHeader1;

// Module-scope variables
int CAN_Timer = 0;
static uint16_t control;

/* ================================================================
 *  Private helpers
 * ================================================================ */

static int16_t maxint16(int16_t a, int16_t b)
{
  return a > b ? a : b;
}

static void CAN1_SetMsg(FDCAN_TxHeaderTypeDef *pTxHeader, const uint8_t *pTxData)
{
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, pTxHeader, pTxData);
  /* 启动FDCAN模块 */
  HAL_FDCAN_Start(&hfdcan1);
}

static void set_time (uint8_t hr, uint8_t min, uint8_t sec)
{
	RTC_TimeTypeDef sTime = {0};
	sTime.Hours = hr;
	sTime.Minutes = min;
	sTime.Seconds = sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}

static void set_date (uint8_t year, uint8_t month, uint8_t date, uint8_t day)  // monday = 1
{
	RTC_DateTypeDef sDate = {0};
	sDate.WeekDay = day;
	sDate.Month = month;
	sDate.Date = date;
	sDate.Year = year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x2345);  // backup register
}

/* ================================================================
 *  FDCAN1 Configuration
 * ================================================================ */

void Config_Fdcan1(void)
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
  // FDCAN_FilterTypeDef CAN1RxFilterConfig;
  CAN1RxFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN1RxFilterConfig.FilterIndex = 1;
  CAN1RxFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  CAN1RxFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  CAN1RxFilterConfig.FilterID1 = 0x100;
  CAN1RxFilterConfig.FilterID2 = 0x7FF;
  CAN1RxFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1,&CAN1RxFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  #ifdef CAN_CONFIG
  CAN1RxFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN1RxFilterConfig.FilterIndex = 2;
  CAN1RxFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  CAN1RxFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  CAN1RxFilterConfig.FilterID1 = 0x200;
  CAN1RxFilterConfig.FilterID2 = 0x7FF;
  CAN1RxFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1,&CAN1RxFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  #endif

  CAN1RxFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN1RxFilterConfig.FilterIndex = 3;
  CAN1RxFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  CAN1RxFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  CAN1RxFilterConfig.FilterID1 = 0x720+MOT_ID;
  CAN1RxFilterConfig.FilterID2 = 0x7FF;
  CAN1RxFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1,&CAN1RxFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_FULL,0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_BUS_OFF,0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_FDCAN_Start(&hfdcan1);
}

/* ================================================================
 *  TX Functions
 * ================================================================ */

void CAN_Send_State(uint16_t DCV, int16_t DCA)
{
  uint8_t StateData[4];
  StateData[0] = DCV;
  StateData[1] = DCV >> 8;
  StateData[2] = DCA;
  StateData[3] = DCA >> 8;
  CAN1_SetMsg(&StateHeader,StateData);
}

void CAN_Send_Status(uint16_t report_status ,int16_t FB_Torque, int16_t Speed)
{
  uint8_t StatusData[6];
  StatusData[0] = report_status;
  StatusData[1] = error_state;
  StatusData[2] = FB_Torque;
  StatusData[3] = FB_Torque >> 8;
  StatusData[4] = Speed;
  StatusData[5] = Speed >> 8;
  // StatusData[6] = error_state;
  CAN1_SetMsg(&StatusHeader,StatusData);
}

void CAN_Send_Temp(uint16_t ADC_arr[6])
{
  uint8_t TempData[6];
  telem.T_Mot = Mot_Conv[ADC_arr[2]>>6];
  telem.T_MCU = MCU_Conv[ADC_arr[1]>>6];
  telem.T_U = Inv_Conv[ADC_arr[3]>>6];
  telem.T_V = Inv_Conv[ADC_arr[4]>>6];
  telem.T_W = Inv_Conv[ADC_arr[5]>>6];
  telem.T_Report = maxint16(telem.T_U, maxint16(telem.T_V, telem.T_W));
  //OTP
  if(maxint16(telem.T_Report, telem.T_MCU) > MOS_OTP)
  {
    // Enter_ERROR_State(ERROR_INV_OT);
  }
  if(telem.T_Mot > MOT_OTP || telem.T_Mot < MOT_UTP)
  {
    #ifndef DISABLE_MOT_OT
    Enter_ERROR_State(ERROR_MOT_OT);
    #endif
  }
  TempData[0] = telem.T_Report;
  TempData[1] = telem.T_Report >> 8;
  TempData[2] = telem.T_MCU;
  TempData[3] = telem.T_MCU >> 8;
  TempData[4] = telem.T_Mot;
  TempData[5] = telem.T_Mot >> 8;
  CAN1_SetMsg(&TempHeader, TempData);
}

void CAN_Send_Heartbeat(void)
{
  uint8_t HBData = 0x05;
  CAN1_SetMsg(&HeartBeatHeader, &HBData);
}

void CAN_Send_Perameter(void)
{
  static uint8_t count;
  count++;
  if(count == 9)
  {
    count = 0;
  }
  uint8_t PeramData[5] = {0};
  PeramData[0] = count+1;
  uint8_t bytes[4] = {0};
  float val = 0.0f;
  uint8_t head = 0x0;
  switch(count)
  {
    case 0:
      head = 0x1;
      val = pid_controller_current_Iq.P;
      break;

    case 1:
      head = 0x2;
      val = pid_controller_current_Iq.I;
      break;

    case 2:
      head = 0x3;
      val = pid_controller_current_Iq.D;
      break;

    case 3:
      head = 0x11;
      val = pid_controller_current_Id.P;
      break;

    case 4:
      head = 0x12;
      val = pid_controller_current_Id.I;
      break;

    case 5:
      head = 0x13;
      val = pid_controller_current_Id.D;
      break;

    case 6:
      head = 0x21;
      val = pid_controller_current_Iabc[0].P;
      break;

    case 7:
      head = 0x22;
      val = pid_controller_current_Iabc[0].I;
      break;

    case 8:
      head = 0x23;
      val = pid_controller_current_Iabc[0].D;
      break;

    default:
      break;
  }
  PeramData[0] = head;
  PeramData[0] = count+1;
  memcpy(&bytes,&val,4);
  for (size_t i = 0; i < 4; i++)
  {
    PeramData[i+1] = bytes[i];
  }
  CAN1_SetMsg(&PerameterHeader,PeramData);
}

/* ================================================================
 *  HAL CAN Callbacks (RX & Error)
 * ================================================================ */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=0)
  {
    HAL_GPIO_TogglePin(LED_CAN_GPIO_Port,LED_CAN_Pin);
    uint8_t RxData1[6];
    HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&RxHeader1,RxData1);
    if (hfdcan->Instance == FDCAN1)
    {
      if (RxHeader1.Identifier == CAN_ID_CONTROL+MOT_ID)
      {
        int16_t torque_command;
        control = RxData1[0] | (uint16_t)RxData1[1] << 8;
        if(inverter_state != STATE_ERROR)
        {
          // enable
          if (control & CTRL_ENABLE && motor.voltage_power_supply >= 20 && inverter_state == STATE_READY && HAL_GPIO_ReadPin(GATE_Ready_GPIO_Port,GATE_Ready_Pin) ==
GPIO_PIN_SET )
          {
            inverter_state = STATE_RUNNING;
            indexMusic = 0;
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_SET);
            foc.percent_torque_requested = 0;
            foc.enable_hw_oc = 1;
            for (size_t i = 0; i < HW_OC_TIME; i++)
            {
              oc_buf[i] = 0;
            }
            oc_sum = 0;
            oc_index = 0;

            soft_oc_sum = 0;
            soft_oc_index = 0;
            for (size_t i = 0; i < SOFT_OC_TIME; i++)
            {
              soft_oc_buf[i] = 0;
            }

            enc_sum = 0;
            enc_index = 0;
            for (size_t i = 0; i < ENC_TIME; i++)
            {
              enc_buf[i] = 0;
            }
            #ifdef FW_STARTUP_ID_FIX
            if (fabsf(foc.filtered_RPM) > 6000.0f)
            {
              filter_Idfw.y_prev = -40.0f;
            }
            #endif

            HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
          // disable
          } else if (!(control & CTRL_ENABLE) && inverter_state == STATE_RUNNING)
          {
            inverter_state = STATE_READY;
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
            foc.percent_torque_requested = 0;
            foc.enable_hw_oc = 0;
            HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
            // got_date = 0;
          }
          else
          {
            foc.enable_hw_oc = 0;
            HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
          }
        }
        // fault reset
        if((control & CTRL_FAULT_RESET) && inverter_state == STATE_ERROR )
        {
          inverter_state = STATE_READY;
          error_state = ERROR_NONE;
        }
        torque_command = RxData1[2] | RxData1[3] << 8;
        foc.percent_torque_requested = (float)torque_command/1000;
        CAN_Timer = 0;
      }
    }
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)!=0)
  {
    HAL_GPIO_TogglePin(LED_CAN_GPIO_Port,LED_CAN_Pin);
    uint8_t RxData1[16];
    HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1,&RxHeader1,RxData1);
    if(hfdcan->Instance == FDCAN1)
    {
      if(RxHeader1.Identifier == 0x100)
      {
        if(got_date)
        {
          return;
        }
        volatile uint32_t sec_from_midnight = RxData1[0] | RxData1[1] << 8 | RxData1[2] << 16 | RxData1[3] << 24;
        volatile uint16_t day_from_1984 = RxData1[4] | RxData1 [5] << 8;
        sec_from_midnight/=1000;
        time_t now = day_from_1984*86400+sec_from_midnight+441763200+28800;
        struct tm now_tm;
        gmtime_r(&now,&now_tm);

        set_time(now_tm.tm_hour,now_tm.tm_min,now_tm.tm_sec);
        set_date(now_tm.tm_year-100,now_tm.tm_mon+1,now_tm.tm_mday,now_tm.tm_wday);
        got_date = 1;
      }
      else if (RxHeader1.Identifier == 0x200)
      {
        if(inverter_state == STATE_RUNNING)
        {
          return;
        }
        float val;
        uint8_t bytes[4] = {RxData1[1],RxData1[2],RxData1[3],RxData1[4]};
        memcpy(&val,&bytes,sizeof(val));
        switch (RxData1[0])
        {
        case 0x1:
          pid_controller_current_Iq.P = val;
          break;

        case 0x2:
          pid_controller_current_Iq.I = val;
          break;

        case 0x3:
          pid_controller_current_Iq.D = val;
          break;

        case 0x11:
          pid_controller_current_Id.P = val;
          break;

        case 0x12:
          pid_controller_current_Id.I = val;
          break;

        case 0x13:
          pid_controller_current_Id.D = val;
          break;

        case 0x21:
          for (size_t i = 0; i < 3; i++)
          {
            pid_controller_current_Iabc[i].P = val;
          }
          break;

        case 0x22:
          for (size_t i = 0; i < 3; i++)
          {
            pid_controller_current_Iabc[i].I = val;
          }
          break;

        case 0x23:
          for (size_t i = 0; i < 3; i++)
          {
            pid_controller_current_Iabc[i].D = val;
          }
          break;

        case 0x31:
          motor.zero_electric_angle = val;
          break;

        case 0x41:
          foc.fast_stop_enable = (uint8_t) val;
          break;

        default:
          break;
        }
      }
      else if (RxHeader1.Identifier == 0x720+MOT_ID)
      {
        float val;
        uint8_t bytes[4] = {RxData1[1],RxData1[2],RxData1[3],RxData1[4]};
        memcpy(&val,&bytes,sizeof(val));
        switch (RxData1[0])
        {
        case 0x21:
          Mot_Curr = val;
          for (size_t i = 0; i < 1024; i++)
          {
            Mot_Conv[i] = (int16_t)10*((float)(1650-(3300*i/1024))/Mot_Curr/3.795-1000/3.795);
          }

        default:
          break;
        }
      }
    }
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if (hfdcan->Instance == FDCAN1)
  {
    HAL_GPIO_TogglePin(LED_CAN_ERR_GPIO_Port,LED_CAN_ERR_Pin);
    MX_FDCAN1_Init();
    Config_Fdcan1();
  }
}