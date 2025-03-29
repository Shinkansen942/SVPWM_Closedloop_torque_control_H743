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
#include "memorymap.h"
#include "rtc.h"
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
#include "logger.h"
// #include "stm32h7xx_hal_tim_ex.h"
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
void Enter_ERROR_State(INV_Errortypedef error);
void Config_Fdcan1(void);
void CAN_Send_State(uint16_t DCV, int16_t DCA);
void CAN_Send_Status(uint16_t report_status,int16_t FB_Torque,int16_t Speed);
void CAN_Send_Temp(void);
void CAN_Send_Heartbeat(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char data[500];
__attribute__((section("._RAM_Area"))) static char tdata[500];

__attribute__((section("._RAM_Area"))) FATFS SDFatFS_RAM;  /* File system object for SD card logical drive */
__attribute__((section("._RAM_Area"))) FIL MyFile;     /* File object */

//Logging Buffers
__attribute__((section("._RAM_Area"))) logger_t log_buf[2][7500];
uint8_t wr_log_buf_num = 0;
uint16_t wr_log_index = 0;
RTC_DateTypeDef log_date;
RTC_TimeTypeDef log_time;
uint8_t last_sec = 0;
uint16_t log_subsec = 0;

//Moving RMS Buffers
int16_t RMS_buf[3][10000] = {{0}};
uint32_t RMS_sum[3] = {0};
uint16_t indexRMS = 0;

int16_t T_Report=0;
int16_t T_Mot = 0;
int16_t T_MCU = 0;
int16_t T_U;
int16_t T_V;
int16_t T_W;
uint16_t report_DCV;

//FOC variables
float open_loop_timestamp=0;
float zero_electric_angle=5.631659;
float shaft_angle=0;
float voltage_limit=24;
float voltage_power_supply=24;
int period=5217; // period for the PWM
int dir=1; // anti clockwise direction is 1 , clockwise is -1
int pole_pairs=1;
float angle_now;
const float ACAPLSB = -0.0515718f;   // ACAPLSB = 3.3/15.626e-3/adc1_range
const float omega_fieldweaking = 3000.0f;
float filtered_RPM;
float filtered_Iq;
float filtered_Id;
float Ia;
float Iq_controller_output;
float Id_controller_output;
float Ia_controller_output;

int indexLED=0;
int indexHeartbeat=0;
int indexStatus=0;
int indexTimer = 0;
int prevSD = 0;
int prevWhileTest = 0;

float Ts=(float)1/10000;
float angle_prev=-1.0f;
const float torque_constant = 0.492f; //Nm/A
const float max_torque = 25;
float percent_torque_requested = 0.02f;
uint16_t current_offset[4];
float current_phase[3];
const int16_t Mot_Conv[1024] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,14,22,31,39,48,56,65,73,82,90,99,107,116,124,133,141,150,158,167,175,184,192,201,209,218,226,235,243,252,260,269,277,286,294,303,311,320,328,337,345,354,362,371,379,388,396,405,413,422,430,439,447,455,464,472,481,489,498,506,515,523,532,540,549,557,566,574,583,591,600,608,617,625,634,642,651,659,668,676,685,693,702,710,719,727,736,744,753,761,770,778,787,795,804,812,821,829,838,846,855,863,872,880,889,897,906,914,923,931,940,948,957,965,973,982,990,999,1007,1016,1024,1033,1041,1050,1058,1067,1075,1084,1092,1101,1109,1118,1126,1135,1143,1152,1160,1169,1177,1186,1194,1203,1211,1220,1228,1237,1245,1254,1262,1271,1279,1288,1296,1305,1313,1322,1330,1339,1347,1356,1364,1373,1381,1390,1398,1407,1415,1424,1432,1441,1449,1458,1466,1475,1483,1491,1500,1508,1517,1525,1534,1542,1551,1559,1568,1576,1585,1593,1602,1610,1619,1627,1636,1644,1653,1661,1670,1678,1687,1695,1704};
const int16_t Inv_Conv[1024] = {-750,-696,-608,-553,-512,-479,-452,-428,-407,-388,-370,-355,-340,-326,-314,-301,-290,-279,-269,-259,-250,-241,-232,-224,-216,-208,-200,-193,-186,-179,-172,-166,-160,-153,-147,-142,-136,-130,-125,-119,-114,-109,-104,-99,-94,-89,-84,-80,-75,-71,-66,-62,-58,-53,-49,-45,-41,-37,-33,-29,-26,-22,-18,-14,-11,-7,-4,0,2,6,9,13,16,19,22,26,29,32,35,38,41,44,47,50,53,56,59,62,65,68,71,73,76,79,82,84,87,90,92,95,98,100,103,105,108,110,113,115,118,120,123,125,128,130,132,135,137,139,142,144,146,149,151,153,155,158,160,162,164,167,169,171,173,175,177,179,182,184,186,188,190,192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,231,233,235,237,239,241,243,245,246,248,250,252,254,256,257,259,261,263,265,266,268,270,272,273,275,277,279,280,282,284,286,287,289,291,292,294,296,297,299,301,302,304,306,307,309,311,312,314,316,317,319,321,322,324,325,327,329,330,332,333,335,337,338,340,341,343,345,346,348,349,351,352,354,355,357,359,360,362,363,365,366,368,369,371,372,374,375,377,378,380,381,383,384,386,387,389,390,392,393,395,396,398,399,401,402,404,405,407,408,409,411,412,414,415,417,418,420,421,422,424,425,427,428,430,431,432,434,435,437,438,440,441,442,444,445,447,448,449,451,452,454,455,456,458,459,461,462,463,465,466,468,469,470,472,473,474,476,477,479,480,481,483,484,485,487,488,490,491,492,494,495,496,498,499,501,502,503,505,506,507,509,510,511,513,514,515,517,518,519,521,522,523,525,526,528,529,530,532,533,534,536,537,538,540,541,542,544,545,546,548,549,550,552,553,554,556,557,558,560,561,562,564,565,566,568,569,570,572,573,574,576,577,578,580,581,582,583,585,586,587,589,590,591,593,594,595,597,598,599,601,602,603,605,606,607,609,610,611,613,614,615,617,618,619,621,622,623,625,626,627,629,630,631,633,634,635,636,638,639,640,642,643,644,646,647,648,650,651,652,654,655,656,658,659,660,662,663,664,666,667,668,670,671,672,674,675,677,678,679,681,682,683,685,686,687,689,690,691,693,694,695,697,698,699,701,702,703,705,706,708,709,710,712,713,714,716,717,718,720,721,723,724,725,727,728,729,731,732,733,735,736,738,739,740,742,743,745,746,747,749,750,751,753,754,756,757,758,760,761,763,764,765,767,768,770,771,772,774,775,777,778,780,781,782,784,785,787,788,789,791,792,794,795,797,798,800,801,802,804,805,807,808,810,811,813,814,815,817,818,820,821,823,824,826,827,829,830,832,833,835,836,838,839,840,842,843,845,846,848,849,851,852,854,856,857,859,860,862,863,865,866,868,869,871,872,874,875,877,878,880,882,883,885,886,888,889,891,893,894,896,897,899,900,902,904,905,907,908,910,912,913,915,916,918,920,921,923,925,926,928,929,931,933,934,936,938,939,941,943,944,946,948,949,951,953,954,956,958,959,961,963,965,966,968,970,971,973,975,977,978,980,982,984,985,987,989,991,992,994,996,998,1000,1001,1003,1005,1007,1009,1010,1012,1014,1016,1018,1020,1021,1023,1025,1027,1029,1031,1033,1034,1036,1038,1040,1042,1044,1046,1048,1050,1052,1054,1055,1057,1059,1061,1063,1065,1067,1069,1071,1073,1075,1077,1079,1081,1083,1085,1087,1089,1091,1093,1095,1097,1100,1102,1104,1106,1108,1110,1112,1114,1116,1118,1121,1123,1125,1127,1129,1131,1134,1136,1138,1140,1142,1145,1147,1149,1151,1154,1156,1158,1160,1163,1165,1167,1169,1172,1174,1176,1179,1181,1184,1186,1188,1191,1193,1195,1198,1200,1203,1205,1208,1210,1213,1215,1218,1220,1223,1225,1228,1230,1233,1235,1238,1241,1243,1246,1248,1251,1254,1256,1259,1262,1264,1267,1270,1273,1275,1278,1281,1284,1287,1289,1292,1295,1298,1301,1304,1307,1310,1312,1315,1318,1321,1324,1327,1330,1333,1337,1340,1343,1346,1349,1352,1355,1358,1362,1365,1368,1371,1375,1378,1381,1385,1388,1391,1395,1398,1402,1405,1409,1412,1416,1419,1423,1426,1430,1434,1437,1441,1445,1448,1452,1456,1460,1464,1468,1471,1475,1479,1483,1487,1491,1496,1500,1504,1508,1512,1516,1521,1525,1529,1534,1538,1542,1547,1551,1556,1561,1565,1570,1575,1579,1584,1589,1594,1599,1604,1609,1614,1619,1624,1629,1635,1640,1645,1651,1656,1662,1667,1673,1678,1684,1690,1696,1702,1708,1714,1720,1726,1733,1739,1745,1752,1758,1765,1772,1779,1785,1792,1800,1807,1814,1821,1829,1836,1844,1852,1860,1868,1876,1884,1892,1901,1909,1918,1927,1936,1945,1954,1964,1973,1983,1993,2003,2014,2024,2035,2046,2057,2068,2080,2091,2103,2116,2128,2141,2154,2168,2181,2195,2210,2224,2239,2255,2271,2287,2304,2321,2338,2356,2375,2394,2414,2435,2456,2478,2501,2524,2549,2574,2600,2628,2657,2686,2718,2750,2785,2821,2859,2899,2942,2987,3035,3087,3142,3202,3266,3336,3412,3497,3590,3695,3814,3951,4111,4301,4534,4830,5227,5804,6778,9092};
int16_t MCU_Conv[1024] = {0};
const float DCVPLSB = 0.0098;     // DCVPLSB = 451*3.3/adc3_range 
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
int max_sd_buf = 0;
#endif

INV_Statustypedef inverter_state = STATE_INIT;
INV_Errortypedef error_state = ERROR_NONE;

__attribute__((section("._ADC1_Area"))) uint16_t ADC1_arr[4] = {0};
__attribute__((section("._ADC2_Area"))) uint16_t ADC2_arr[4] = {0};
__attribute__((section("._ADC3_Area"))) uint16_t ADC3_arr[6] = {0};

//Temperature Constants
float ts_cal1;
float ts_cal2;

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
FDCAN_RxHeaderTypeDef RxHeader1;
// uint32_t can_txbuf_num = 0x1u;

int isSent = 1;
uint16_t control;
int CAN_Timer = 0;

const char TestFPath[] = {"Test.txt"};
const char TextFPath[] = {"Text.bin"};

lpf_t filter= {.Tf=0.0001,.y_prev=0.0f};         //Tf=1ms
lpf_t filter_current_Iq= {.Tf=0.002,.y_prev=0.0f}; //Tf=5ms
lpf_t filter_current_Id= {.Tf=0.002,.y_prev=0.0f}; //Tf=5ms
lpf_t filter_current_Iabc[3] = {{.Tf = 0.002,.y_prev=0.0f},{.Tf = 0.002,.y_prev=0.0f},{.Tf = 0.002,.y_prev=0.0f}}; //Tf=2ms
lpf_t filter_RPM = {.Tf=0.05,.y_prev=0.0f};
pid_t pid_controller_current_Iq = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pid_t pid_controller_current_Id = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pid_t pid_controller_current_OCP = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pid_t pid_controller_current_Ia = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};

int16_t maxint16(int16_t a,int16_t b)
{
  return a>b?a:b;
}

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

void CAN1_SetMsg(FDCAN_TxHeaderTypeDef *pTxHeader, const uint8_t *pTxData)
{
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, pTxHeader, pTxData);
    /* 启动FDCAN模块 */
    HAL_FDCAN_Start(&hfdcan1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  volatile FRESULT res;                                 /* FatFs function common result code */
	uint32_t byteswritten, bytesread;                     /* File write/read counts */
	uint8_t wtext[] = "This is STM32 working with FatFs\n"; /* File write buffer */
  pid_controller_current_Ia.limit = voltage_limit;
  pid_controller_current_Id.limit = voltage_limit;
  pid_controller_current_Iq.limit = voltage_limit;
  pid_controller_current_OCP.limit = voltage_limit;
  // uint8_t wlooptext[] = "This is STM32 working with FatFs in main loop\n"; /* File write buffer */
  // log_buf[0][0].LGSTATE = 0;
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
  MX_SPI4_Init();
  MX_RTC_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_D13_GPIO_Port,LED_D13_Pin,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(LED_D12_GPIO_Port,LED_D12_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_D11_GPIO_Port,LED_D11_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_D10_GPIO_Port,LED_D10_Pin,GPIO_PIN_RESET);

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

  #ifdef SDDEBUG
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
      res = f_open(&MyFile,TestFPath,FA_CREATE_ALWAYS | FA_WRITE);
      if (res != FR_OK)
      {
        Error_Handler();
      }
      else
      {
        f_write(&MyFile,wtext,sizeof(wtext),(void *)&byteswritten);
        res = f_close(&MyFile);
        if(res!=FR_OK)
        {
          Error_Handler();
        }
      }
    }
  }
  #endif

  // Init ADC DMA
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_arr,4);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_arr,4);
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)ADC3_arr,6);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel0_dma1_stream2_tc_0,(uint32_t)tmp_ADC1_arr,(uint32_t)ADC1_arr,8,1);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel1_dma1_stream1_tc_0,(uint32_t)tmp_ADC2_arr,(uint32_t)ADC2_arr,8,1);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel2_dma1_stream4_tc_0,(uint32_t)tmp_ADC3_arr,(uint32_t)ADC3_arr,12,1);
  // HAL_ADC_Start_IT(&hadc3);

  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_Delay(1000);
  calibrateOffsets(current_offset,ADC1_arr);
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);

  //Wait for GATE READY Signal
  #ifdef WAIT_GATE_READY
  while (HAL_GPIO_ReadPin(GATE_Ready_GPIO_Port,GATE_Ready_Pin) != GPIO_PIN_SET)
  {
    HAL_Delay(1);
  }
  #endif
   
  #ifdef CAL_ZERO_ANGLE
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
  setPhaseVoltage(10,0,_electricalAngle(M_PI*1.5f,pole_pairs),TIM1);
  HAL_Delay(3000);
  // uint16_t read_raw=read(&hspi1, SPI1_CSn_GPIO_Port,SPI1_CSn_Pin,AS5048A_ANGLE);
  float raw_angle;
  Get_Encoder_Angle(ADC2_arr,&raw_angle);
  zero_electric_angle=_electricalAngle(raw_angle,pole_pairs);
  setPhaseVoltage(0,0,_electricalAngle(M_PI*1.5f,pole_pairs),TIM1);
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  #endif

  UART_TX_Send(&huart1,"zero_electric_angle: %i \n",(int) floor(zero_electric_angle/M_PI*180));
  
  Config_Fdcan1();

  //Get DateTime
  HAL_RTC_GetDate(&hrtc,&log_date,RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc,&log_time,RTC_FORMAT_BIN);

  res = f_open(&MyFile,TextFPath,FA_CREATE_ALWAYS|FA_WRITE);
  if(res == FR_OK)
  {
    f_close(&MyFile);
  }
  f_open(&MyFile,TextFPath,FA_OPEN_APPEND|FA_WRITE);

  HAL_GPIO_WritePin(LED_D13_GPIO_Port,LED_D13_Pin,GPIO_PIN_RESET);
  inverter_state = STATE_READY;
  error_state = ERROR_NONE;

  #ifdef TIMING
  prev_time = __HAL_TIM_GET_COUNTER(&htim5);
  #endif

  HAL_GPIO_WritePin(LED_D11_GPIO_Port,LED_D11_Pin,GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim3); 
  prevSD = __HAL_TIM_GET_COUNTER(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    volatile int sd_now = __HAL_TIM_GET_COUNTER(&htim2);
    // uint32_t whileTest = sd_now;
    if (sd_now - prevSD >= 1000)
    {
      HAL_GPIO_WritePin(LED_D11_GPIO_Port,LED_D11_Pin,GPIO_PIN_RESET);
      
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
      f_sync(&MyFile);

      #ifdef TIMING
      sd_td = __HAL_TIM_GET_COUNTER(&htim2) - sd_now;
      if (max_sdwrite < sd_td)
      {
        max_sdwrite = sd_td;
      }      
      prevSD = sd_now;
      #endif

      HAL_GPIO_WritePin(LED_D11_GPIO_Port,LED_D11_Pin,GPIO_PIN_SET);
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
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//   if(hadc->Instance == hadc3.Instance) {
//     volatile uint32_t adc_val = HAL_ADC_GetValue(hadc);
//     volatile int a = 0;
//   }
// }


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim3 )
  {
    HAL_GPIO_TogglePin(LED_D8_GPIO_Port,LED_D8_Pin);
    SCB_InvalidateDCache_by_Addr(ADC1_arr,sizeof(ADC1_arr));
    SCB_InvalidateDCache_by_Addr(ADC2_arr,sizeof(ADC2_arr));
    SCB_InvalidateDCache_by_Addr(ADC3_arr,sizeof(ADC3_arr));
    if(inverter_state == STATE_RUNNING)
    {
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
    }
    if(inverter_state == STATE_READY)
    {
      PID_reset(&pid_controller_current_Id);
      PID_reset(&pid_controller_current_Iq);
      PID_reset(&pid_controller_current_OCP);
      PID_reset(&pid_controller_current_Ia);
    }
    
    #ifdef TIMING
    uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim5);     
    #endif

    indexLED++;
    indexHeartbeat++;
    indexStatus++;
    CAN_Timer++;
    

    Get_Encoder_Angle(ADC2_arr,&angle_now);
    for (size_t i = 0; i < 4; i++)
    {
      if(ADC2_arr[i] < ENC_UV)
      {
        Enter_ERROR_State(ERROR_ENC);
      }
    }
    

    float angular_vel = cal_angular_vel(angle_now);
    float filtered_vel = LowPassFilter_operator(angular_vel,&filter);
    filtered_RPM = LowPassFilter_operator((float)dir*filtered_vel/4/2/M_PI*60,&filter_RPM);
    float target_torque = max_torque*percent_torque_requested;
    float target_Iq = target_torque/torque_constant;
    float filtered_Iabc[3] = {0.0f};
    for (int i=0;i<3;i++){
       	current_phase[i] =(float) (ADC1_arr[i]-current_offset[i])*ACAPLSB;
        //OCP
        filtered_Iabc[i] = LowPassFilter_operator(current_phase[i],&filter_current_Iabc[i]);
        if (filtered_Iabc[i] > ACAOCP||current_phase[i] < -ACAOCP)
        {
          Enter_ERROR_State(ERROR_INSTANT_OC);
        }
    }

    float Id,Iq;
    cal_Idq(current_phase, _electricalAngle(angle_now, pole_pairs), &Id, &Iq);
    filtered_Iq=LowPassFilter_operator(Iq,&filter_current_Iq);
    filtered_Id=LowPassFilter_operator(Id,&filter_current_Id);
    Ia = sqrt(filtered_Id*filtered_Id+filtered_Iq*filtered_Iq);

    #ifndef SIXSTEP
    Iq_controller_output=PID_operator(target_Iq-filtered_Iq,&pid_controller_current_Iq);
    Id_controller_output=PID_operator(-filtered_Id,&pid_controller_current_Id);
    float IqOC_controller_output;
    if(Iq_controller_output > 0)
    {
      IqOC_controller_output = _constrain(PID_operator(SOFTOCP-Ia,&pid_controller_current_OCP),-Iq_controller_output,0);
    }
    else
    {
      IqOC_controller_output = _constrain(PID_operator(-SOFTOCP-Ia,&pid_controller_current_OCP),0,-Iq_controller_output);
    }
    

    // setPhaseVoltage(_constrain(Iq_controller_output+IqOC_controller_output,-voltage_power_supply/2,voltage_power_supply/2),  _constrain(Id_controller_output,-voltage_power_supply/2,voltage_power_supply/2), _electricalAngle(angle_now, pole_pairs),TIM1);
    // setPhaseVoltage(percent_torque_requested*100, 0, _electricalAngle(angle_now, pole_pairs),TIM1);
    setPhaseVoltage(Iq_controller_output, Id_controller_output, _electricalAngle(angle_now, pole_pairs),TIM1);
    #endif

    #ifdef SIXSTEP
    Ia_controller_output=PID_operator(target_Iq-Ia,&pid_controller_current_Ia);
    setSixStepPhaseVoltage(-Ia_controller_output,_electricalAngle(angle_now,pole_pairs),TIM1);
    #endif

    if (indexLED == 5000)
    {
    	if(inverter_state == STATE_READY)
      {
        HAL_GPIO_TogglePin(LED_D10_GPIO_Port, LED_D10_Pin);
        HAL_GPIO_WritePin(LED_D13_GPIO_Port,LED_D13_Pin,GPIO_PIN_RESET);
      }
      else if(inverter_state == STATE_RUNNING)
      {
        HAL_GPIO_WritePin(LED_D10_GPIO_Port,LED_D10_Pin,GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_D13_GPIO_Port,LED_D13_Pin,GPIO_PIN_RESET);
        error_state = ERROR_NONE;
      }
      else if(inverter_state == STATE_ERROR)
      {
        HAL_GPIO_WritePin(LED_D10_GPIO_Port,LED_D10_Pin,GPIO_PIN_RESET);
        HAL_GPIO_TogglePin(LED_D13_GPIO_Port,LED_D13_Pin);
      }
      
    	// HAL_GPIO_TogglePin(LED_D13_GPIO_Port, LED_D13_Pin);
      // UART_TX_Send(&huart1,"ping");
    	indexLED=0;
    }

    report_DCV = (uint16_t) roundf((float)ADC3_arr[0]*DCVPLSB*100);
    int16_t report_DCA = (int16_t) roundf((float)(ADC1_arr[3]-current_offset[3])*DCAPLSB*100);
    if (indexHeartbeat == 1000)
    {
      CAN_Send_Temp();
      CAN_Send_State(report_DCV,report_DCA);
      CAN_Send_Heartbeat();
      // CAN_Send_Heartbeat();
      indexHeartbeat = 0;
    }

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

    if (indexStatus == 100)
    {
      int16_t report_RPM = (int16_t) roundf(filtered_vel/2/M_PI*60/4);
      int16_t report_torque = (int16_t) roundf(Ia*torque_constant/max_torque*1000);
      CAN_Send_Status(report_status,report_torque,report_RPM);
      indexStatus = 0;
    }

    //Logging
    int16_t IU_100 = (int16_t)roundf(current_phase[1]*100);
    int16_t IV_100 = (int16_t)roundf(current_phase[2]*100);
    int16_t IW_100 = (int16_t)roundf(current_phase[3]*100);
    HAL_RTC_GetDate(&hrtc,&log_date,RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc,&log_time,RTC_FORMAT_BIN);
    if(log_time.Seconds != last_sec)
    {
      log_subsec = 0;
      last_sec = log_time.Seconds;
    }
    log_buf[wr_log_buf_num][wr_log_index%7500].LGHR = log_time.Hours;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGMIN = log_time.Minutes;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGSEC = log_time.Seconds;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGSUBSEC = log_subsec;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGDCV = report_DCV;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGDCA = report_DCA;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGIU = IU_100;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGIV = IV_100;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGIW = IW_100;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGTMOS = T_Report;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGTMOT = T_Mot;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGSINE = ADC2_arr[0]-ADC2_arr[1];
    log_buf[wr_log_buf_num][wr_log_index%7500].LGCOS = ADC2_arr[2]-ADC2_arr[3];
    log_buf[wr_log_buf_num][wr_log_index%7500].LGANG = (uint16_t) roundf(angle_now*100*180/M_PI);
    log_buf[wr_log_buf_num][wr_log_index%7500].LGTCMD = (int16_t) roundf(percent_torque_requested*10);
    log_buf[wr_log_buf_num][wr_log_index%7500].LGSTATE = report_status;
    log_buf[wr_log_buf_num][wr_log_index%7500].LGCRC = HAL_CRC_Calculate(&hcrc,&log_buf[wr_log_buf_num][wr_log_index%7500],((sizeof(logger_t)-2)));
        
    log_subsec++;
    wr_log_index++;


    #ifdef RMSOCP
    //Moving RMS for phase currents
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
    #endif

    #ifdef CAN_OT_FAULT
    //CAN fault detect
    if(CAN_Timer == 10000 && inverter_state == STATE_RUNNING)
    {
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
      inverter_state = STATE_READY;
    }
    #endif

    #ifdef TIMING
    loop_time = __HAL_TIM_GET_COUNTER(&htim5)-tick_start;
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
    if (indexTimer == 100000)
    {
      max_time = 0;
      min_time = INT16_MAX;
      max_btw = 0;
      indexTimer = 0;
      max_sdwrite = 0;
    }
    indexTimer++;
    #endif

    HAL_GPIO_TogglePin(LED_D8_GPIO_Port,LED_D8_Pin);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
  if(GPIO_PIN == GPIO_PIN_3)
  {
    Enter_ERROR_State(ERROR_GATE);
  }
  if(GPIO_PIN == GPIO_PIN_6)
  {
    // Enter_ERROR_State(ERROR_HW_OC);
  }
  if(GPIO_PIN == GPIO_PIN_5)
  {
    // f_write(&MyFile,write_buffer,sizeof(write_buffer),&written);
    f_sync(&MyFile);
    while (1)
    {
      /* code */
    }    
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	isSent = 1;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=0)
  {
    HAL_GPIO_TogglePin(LED_B13_GPIO_Port,LED_B13_Pin);
    uint8_t RxData1[4];
    HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&RxHeader1,RxData1);
    if(hfdcan->Instance == FDCAN1)
    {
      int16_t torque_command;
      control = RxData1[0] | (uint16_t)RxData1[1] << 8;
      // enable
      if(control & CTRL_ENABLE && inverter_state == STATE_READY && HAL_GPIO_ReadPin(GATE_Ready_GPIO_Port,GATE_Ready_Pin) == GPIO_PIN_SET)
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
        error_state = ERROR_NONE;
      }
      torque_command = RxData1[2] | RxData1[3] << 8;
      percent_torque_requested = (float)torque_command/1000;
      CAN_Timer = 0;
    }
  }
}

#ifdef CAN_CONFIG
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)!=0)
  {
    HAL_GPIO_TogglePin(LED_B13_GPIO_Port,LED_B13_Pin);
    uint8_t RxData1[4];
    HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1,&RxHeader1,RxData1);
    if(hfdcan->Instance == FDCAN1)
    {
      uint16_t uintdata = RxData1[0] | RxData1[1] << 8;
      zero_electric_angle = 6.28f*uintdata/1000;
    }
  }
}
#endif

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if(hfdcan->Instance == FDCAN1)
  {
    HAL_GPIO_TogglePin(LED_B12_GPIO_Port,LED_B12_Pin);
    MX_FDCAN1_Init();
    Config_Fdcan1();
  }
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

  #ifdef CAN_CONFIG
  CAN1RxFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN1RxFilterConfig.FilterIndex = 0;
  CAN1RxFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  CAN1RxFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  CAN1RxFilterConfig.FilterID1 = CAN_ID_CONFIG+MOT_ID;
  CAN1RxFilterConfig.FilterID2 = 0x7FF;
  CAN1RxFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1,&CAN1RxFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  #endif

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)
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

void Enter_ERROR_State(INV_Errortypedef error)
{
  inverter_state = STATE_ERROR;
  error_state = error;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
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
  StatusData[1] = report_status >> 8;
  StatusData[2] = FB_Torque;
  StatusData[3] = FB_Torque >> 8;
  StatusData[4] = Speed;
  StatusData[5] = Speed >> 8;
  CAN1_SetMsg(&StatusHeader,StatusData);
}

void CAN_Send_Temp(void)
{
  uint8_t TempData[6];
  T_Mot = Mot_Conv[ADC3_arr[2]>>6];
  T_MCU = MCU_Conv[ADC3_arr[1]>>6];
  T_U = Inv_Conv[ADC3_arr[3]>>6];
  T_V = Inv_Conv[ADC3_arr[4]>>6];
  T_W = Inv_Conv[ADC3_arr[5]>>6];
  T_Report = maxint16(T_U,maxint16(T_V,T_W));
  //OTP
  if(maxint16(T_Report,T_MCU) > MOS_OTP)
  {
    Enter_ERROR_State(ERROR_INV_OT);
  }
  if(T_Mot > MOT_OTP)
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
  uint8_t HBData = 0x7f;
  CAN1_SetMsg(&HeartBeatHeader,&HBData);
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
