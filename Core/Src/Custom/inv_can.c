#include "inv_can.h"

// CAN Headers
FDCAN_TxHeaderTypeDef HeartBeatHeader = { .Identifier = CAN_ID_HEARTBEAT+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_1,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x01};
FDCAN_TxHeaderTypeDef TempHeader      = { .Identifier = CAN_ID_TEMPERATURE+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_6,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x02};
FDCAN_TxHeaderTypeDef StateHeader     = { .Identifier = CAN_ID_STATE+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_4,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x03};
FDCAN_TxHeaderTypeDef StatusHeader    = { .Identifier = CAN_ID_STATUS+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_6,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x04};
FDCAN_TxHeaderTypeDef PerameterHeader = { .Identifier = CAN_ID_PERAM+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_8,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x05};
FDCAN_RxHeaderTypeDef RxHeader1;

int16_t maxint16(int16_t a,int16_t b)
{
  return a>b?a:b;
}

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

void CAN1_SetMsg(FDCAN_TxHeaderTypeDef *pTxHeader, const uint8_t *pTxData)
{
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, pTxHeader, pTxData);
    /* 启动FDCAN模块 */
    HAL_FDCAN_Start(&hfdcan1);
}

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
  T_Mot = Mot_Conv[ADC_arr[2]>>6];
  T_MCU = MCU_Conv[ADC_arr[1]>>6];
  T_U = Inv_Conv[ADC_arr[3]>>6];
  T_V = Inv_Conv[ADC_arr[4]>>6];
  T_W = Inv_Conv[ADC_arr[5]>>6];
  T_Report = maxint16(T_U,maxint16(T_V,T_W));
  //OTP
  if(maxint16(T_Report,T_MCU) > MOS_OTP)
  {
    // Enter_ERROR_State(ERROR_INV_OT);
  }
  if(T_Mot > MOT_OTP || T_Mot < MOT_UTP)
  {
    Enter_ERROR_State(ERROR_MOT_OT);
  }
  TempData[0] = T_Report;
  TempData[1] = T_Report >> 8;
  TempData[2] = T_MCU;
  TempData[3] = T_MCU >> 8;
  TempData[4] = T_Mot;
  TempData[5] = T_Mot >> 8;
  CAN1_SetMsg(&TempHeader,TempData);
}

void CAN_Send_Heartbeat(void)
{
  uint8_t HBData = 0x05;
  CAN1_SetMsg(&HeartBeatHeader,&HBData);
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
  PeramData[0] = count;
  uint8_t bytes[4] = {0};
  float val = 0.0f;
  switch(count)
  {
    case 0:
      val = pid_controller_current_Iq.P;
      break;

    case 1:
      val = pid_controller_current_Iq.I;
      break;
    
    case 2:
      val = pid_controller_current_Iq.D;
      break;

    case 3:
      val = pid_controller_current_Id.P;
      break;

    case 4:
      val = pid_controller_current_Id.I;
      break;
    
    case 5:
      val = pid_controller_current_Id.D;
      break;

    case 6:
      val = pid_controller_current_Iabc[0].P;
      break;

    case 7:
      val = pid_controller_current_Iabc[0].I;
      break;

    case 8:
      val = pid_controller_current_Iabc[0].D;
      break;

    default:
      break;
  }
  memcpy(&bytes,&val,4);
  for (size_t i = 0; i < 4; i++)
  {
    PeramData[i+1] = bytes[i];
  }  
  CAN1_SetMsg(&PerameterHeader,PeramData);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=0)
  {
    HAL_GPIO_TogglePin(LED_CAN_GPIO_Port,LED_CAN_Pin);
    uint8_t RxData1[6];
    HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&RxHeader1,RxData1);
    if(hfdcan->Instance == FDCAN1)
    {
      if(RxHeader1.Identifier == CAN_ID_CONTROL+MOT_ID)
      {
        int16_t torque_command;
        uint16_t control;
        control = RxData1[0] | (uint16_t)RxData1[1] << 8;
        if(inverter_state != STATE_ERROR)
        {
          // enable
          if(control & CTRL_ENABLE && voltage_power_supply >= 20 && inverter_state == STATE_READY && HAL_GPIO_ReadPin(GATE_Ready_GPIO_Port,GATE_Ready_Pin) == GPIO_PIN_SET )
          {
            inverter_state = STATE_RUNNING;
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_SET);
            percent_torque_requested = 0;
            enable_hw_oc = 1;
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
            
            
            HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
          // disable
          }else if(!(control & CTRL_ENABLE) && inverter_state == STATE_RUNNING) 
          {
            inverter_state = STATE_READY;
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
            percent_torque_requested = 0;
            enable_hw_oc = 0;
            HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
          }
          else
          {
            enable_hw_oc = 0;
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
        percent_torque_requested = (float)torque_command/1000;
        CAN_Timer = 0;
        soft_oc_sum = 0;
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
        volatile uint32_t sec_from_midnight = RxData1[0] | RxData1[1] << 8 | RxData1[2] << 16 | RxData1[3] << 24;
        volatile uint16_t day_from_1984 = RxData1[4] | RxData1 [5] << 8;
        sec_from_midnight/=1000;
        time_t now = day_from_1984*86400+sec_from_midnight+441763200;
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
          zero_electric_angle = val;
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
            // if (Mot_Conv[i] < 0)
            // {
            //   Mot_Conv[i] = 0;
            // }
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
  if(hfdcan->Instance == FDCAN1)
  {
    HAL_GPIO_TogglePin(LED_CAN_ERR_GPIO_Port,LED_CAN_ERR_Pin);
    MX_FDCAN1_Init();
    Config_Fdcan1();
  }
}