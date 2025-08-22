#include "inv_main.h"
#include "inv_func.h"
#include "inv_can.h"

__attribute__((section("._RAM_Area"))) FATFS  SDFatFS_RAM;  /* File system object for SD card logical drive */
__attribute__((section("._RAM_Area"))) FIL    MyFile;       /* File object */

//Logging Buffers
__attribute__((section("._RAM_Area"))) logger_t log_buf[2][3600];
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

uint8_t run = 0;

uint16_t report_DCV;

#ifdef OPEN_LOOP_SPEED
float open_loop_rpm_var = OPEN_LOOP_RPM;
float last_angle = 0.0f;
#endif

//FOC variables
float open_loop_timestamp=0;
float shaft_angle=0;
float voltage_limit=440;
int period=CCR; // period for the PWM
int dir=1; // anti clockwise direction is 1 , clockwise is -1
// int dir = DIR; // anti clockwise direction is 1 , clockwise is -1
int pole_pairs=1;
float angle_now;
float zero_cross = 0.0f;
const float ACAPLSB = -0.1031436f;   // ACAPLSB = 3.3/15.626e-3/adc1_range
const float omega_fieldweaking = 3000.0f;
float filtered_RPM;
// int int_RPM;
float filtered_Iq;
float filtered_Id;
float Ia;
float Iq_controller_output;
float Id_controller_output;
float Ia_controller_output;
uint8_t enable_dc_control = 0;
const float derate_a = 1/(DERATE_START-DERATE_END);
const float derate_b = DERATE_END/(DERATE_START-DERATE_END); 

int indexLED=0;
int indexHeartbeat=0;
int indexStatus=0;
int indexTimer = 0;
uint32_t prev_new_file = 0;
int prev_sd = 0;
int prevWhileTest = 0;
uint8_t last_got_date = 0;

int freq = FREQ;
float Ts=(float)1/FREQ;
// float angle_prev=-1.0f;
const float torque_constant = 0.291f; //Nm/A
const float electrical_constant = 0.031f; //rad/s/V
const float max_torque = 25;
const float max_current = 70;
float last_percent = 0.0f;
float abs_last_percent = 0.0f;
uint16_t current_offset[4];
float current_phase[3];
const float DCVPLSB = 0.00897;     // DCVPLSB = 451*3.3/adc3_range 
const float DCAPLSB = 0.0402930f;   // DCAPLSB = 3.3/20e-3/adc1_range 
float max_ramp = 1/FREQ/CMD_RAMP_TIME;

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

int isSent = 1;

const char TestFPath[] = {"Test.bin"};
char TextFPath[80];

// arm_biquad_casd_df1_inst_f32 biquad_RPM_filter;
// float32_t biquad_RPM_coeffs[5 * NUM_SECTIONS] = {
//     0.00382204025601966,  0.00764408051203933,  0.00382204025601966,  1.84806000698701,   -0.93557266437005,
//     0.012980445021764,    0.0259608900435281,   0.012980445021764,    1.73883997677972,   -0.821180651084955,
//     0.0195151797095608,   0.0390303594191216,   0.0195151797095608,   1.64878896910318,   -0.726865386321526,
//     0.0240097101643741,   0.0480194203287482,   0.0240097101643741,   1.5778791983566,    -0.652597768725278,
//     0.0266193027158704,   0.0532386054317408,   0.0266193027158704,   1.52555703913352,   -0.597797956624961,
//     0.0274397244578231,   0.0548794489156463,   0.0274397244578231,   1.49114767998067,   -0.561759183683089,
//     0.0263487553432427,   0.0526975106864853,   0.0263487553432427,   1.47409583587729,   -0.543899870025087
// };
// float biquad_RPM_states[4 * NUM_SECTIONS] = {0};
// float biquad_RPM = 0.0f;

arm_fir_instance_f32 fir_RPM_filter;
float32_t fir_RPM_coeffs[NUM_TAPS] = {7.313481183e-05,2.656286961e-05,3.131332778e-05,3.650138751e-05,4.223604992e-05,4.844334762e-05,5.523092841e-05,6.253210449e-05,7.047156396e-05,7.897933392e-05,8.815328329e-05,9.787874296e-05,0.0001082692761,0.0001192694908,0.0001310256048,0.0001433146244,0.0001563065161, 0.000169979714,0.0001842603815, 0.000199207745,0.0002147752093,0.0002309367992,0.0002477022354,0.0002650196548,0.0002829008445,0.0003012612578,0.0003201127402,0.0003393812513,0.0003590719425,0.0003790676128,0.0003993697464,0.0004198956594,0.0004405922955,0.0004613797355,0.0004822277697,0.0005029995227,0.0005236924044,0.0005441572284,0.0005643815966,0.0005842031096,0.0006036003469,0.0006224177196,0.0006406262983,0.0006580550689,0.0006746749277,0.0006903240574,0.0007049577544,0.0007184147253,0.0007306668558, 0.000741525495,0.0007509771967,0.0007588416338,0.0007650969783,0.0007695715758,0.0007722416194,0.0007729508798,0.0007716801483,0.0007682772703,0.0007627345622,0.0007549129659,0.0007448099204,0.0007323015016,0.0007174059865,0.0007000003825,0.0006801217096,0.0006576735759,0.0006326982402,0.0006051238743,0.0005750068231,0.0005422978429, 0.000507074059,0.0004693034862,0.0004290879006,0.0003864161845,0.0003414095554,0.0002940831182,0.0002445846621,0.0001929452555,0.0001393363491,8.381998487e-05,2.658227459e-05,-3.228907735e-05,-9.258588398e-05,-0.0001542016544,-0.0002169036015,-0.0002805684635,-0.0003449405776,-0.0004098799254,-0.0004751155793,-0.0005404875847,-0.0006057086866,-0.0006706095301,-0.0007348905201,-0.0007983671385,-0.000860735774,-0.0009218034684,-0.0009812591597, -0.00103891117,-0.001094443956,-0.001147667994,-0.001198270009,-0.001246065833,-0.001290753251,-0.001332153683, -0.00136997737,-0.001404062263,-0.001434135251,-0.001460050466,-0.001481559128,-0.001498537604,-0.001510761096,-0.001518137287,-0.001520470367,  -0.0015176991,-0.001509663416,-0.001496336306, -0.00147759926,-0.001453461125,-0.001423845184,-0.001388804056,-0.001348303747,-0.001302440767,-0.001251229201,-0.001194810611,-0.001133244717,-0.001066723256,-0.0009953536792,-0.0009193728911,-0.0008389379946,-0.0007543310057,-0.0006657579797,-0.000573544763,-0.0004779415613,-0.0003793193027,-0.0002779685892,-0.0001743005705,-6.864584429e-05,3.854970782e-05,0.0001469226554,0.0002559941786,0.0003653718741,0.0004745535553,0.0005831224844,0.0006905585178,0.0007964281831,0.0009001992294, 0.001001430792, 0.001099584857, 0.001194221084, 0.001284805126, 0.001370902522, 0.001451992663, 0.001527657616, 0.001597396214, 0.001660814509, 0.001717442647, 0.001766918227, 0.001808809931, 0.001842797385, 0.001868495136, 0.001885633334, 0.001893880544, 0.001893025241, 0.001882798853,  0.00186305237, 0.001833588351, 0.001794328913, 0.001745150774, 0.001686053234, 0.001616994152, 0.001538052922, 0.001449272502, 0.001350817736, 0.001242817729, 0.001125524752,0.0009991569677,0.0008640547749,0.0007205260335,0.0005689968821,0.0004098647623, 0.000243641829, 7.08099833e-05,-0.000108035405,-0.0002923306893,-0.000481403491,-0.0006746137515,-0.000871216529,-0.001070503495,-0.001271665562,-0.001473932876,-0.001676440937,-0.001878367621,-0.002078804187,-0.002276886487,-0.002471670974,-0.002662264975, -0.00284770201,-0.003027072176,-0.003199400147,-0.003363771597,-0.003519214923, -0.00366482581,-0.003799650585, -0.00392280845,-0.004033376928,-0.004130513407,-0.004213341977,-0.004281070549,-0.004332883749,-0.004368055146,-0.004385841079,-0.004385594744,-0.004366659094,-0.004328477196,-0.004270489328,-0.004192241002,-0.004093282856,-0.003973271232,-0.003831875278,-0.003668873105,-0.003484060057,-0.003277343232, -0.00304865092,-0.002798024798,-0.002525530988,-0.002231348539,-0.001915686065,-0.001578862197, -0.00122122711,-0.0008432405302,-0.0004453934962,-2.828393554e-05,0.0004074586323,0.0008611020166, 0.001331883832, 0.001818942954, 0.002321390901, 0.002838245826, 0.003368502017,   0.0039110668, 0.004464826547, 0.005028590094, 0.005601149518, 0.006181226112, 0.006767532323, 0.007358717266, 0.007953426801, 0.008550255559, 0.009147799574, 0.009744614363,  0.01033926196,  0.01093027741,  0.01151621062,  0.01209559012,  0.01266697049,  0.01322889514,  0.01377994195,  0.01431868784,  0.01484375075,  0.01535375975,  0.01584739052,  0.01632334106,   0.0167803634,  0.01721724495,  0.01763282716,  0.01802599803,  0.01839571074,  0.01874096878,  0.01906084642,  0.01935447566,  0.01962106861,  0.01985989697,  0.02007031813,  0.02025175467,  0.02040371485,   0.0205257833,  0.02061762847,  0.02067899704,   0.0207097251,   0.0207097251,  0.02067899704,  0.02061762847,   0.0205257833,  0.02040371485,  0.02025175467,  0.02007031813,  0.01985989697,  0.01962106861,  0.01935447566,  0.01906084642,  0.01874096878,  0.01839571074,  0.01802599803,  0.01763282716,  0.01721724495,   0.0167803634,  0.01632334106,  0.01584739052,  0.01535375975,  0.01484375075,  0.01431868784,  0.01377994195,  0.01322889514,  0.01266697049,  0.01209559012,  0.01151621062,  0.01093027741,  0.01033926196, 0.009744614363, 0.009147799574, 0.008550255559, 0.007953426801, 0.007358717266, 0.006767532323, 0.006181226112, 0.005601149518, 0.005028590094, 0.004464826547,   0.0039110668, 0.003368502017, 0.002838245826, 0.002321390901, 0.001818942954, 0.001331883832,0.0008611020166,0.0004074586323,-2.828393554e-05,-0.0004453934962,-0.0008432405302, -0.00122122711,-0.001578862197,-0.001915686065,-0.002231348539,-0.002525530988,-0.002798024798, -0.00304865092,-0.003277343232,-0.003484060057,-0.003668873105,-0.003831875278,-0.003973271232,-0.004093282856,-0.004192241002,-0.004270489328,-0.004328477196,-0.004366659094,-0.004385594744,-0.004385841079,-0.004368055146,-0.004332883749,-0.004281070549,-0.004213341977,-0.004130513407,-0.004033376928, -0.00392280845,-0.003799650585, -0.00366482581,-0.003519214923,-0.003363771597,-0.003199400147,-0.003027072176, -0.00284770201,-0.002662264975,-0.002471670974,-0.002276886487,-0.002078804187,-0.001878367621,-0.001676440937,-0.001473932876,-0.001271665562,-0.001070503495,-0.000871216529,-0.0006746137515,-0.000481403491,-0.0002923306893,-0.000108035405, 7.08099833e-05, 0.000243641829,0.0004098647623,0.0005689968821,0.0007205260335,0.0008640547749,0.0009991569677, 0.001125524752, 0.001242817729, 0.001350817736, 0.001449272502, 0.001538052922, 0.001616994152, 0.001686053234, 0.001745150774, 0.001794328913, 0.001833588351,  0.00186305237, 0.001882798853, 0.001893025241, 0.001893880544, 0.001885633334, 0.001868495136, 0.001842797385, 0.001808809931, 0.001766918227, 0.001717442647, 0.001660814509, 0.001597396214, 0.001527657616, 0.001451992663, 0.001370902522, 0.001284805126, 0.001194221084, 0.001099584857, 0.001001430792,0.0009001992294,0.0007964281831,0.0006905585178,0.0005831224844,0.0004745535553,0.0003653718741,0.0002559941786,0.0001469226554,3.854970782e-05,-6.864584429e-05,-0.0001743005705,-0.0002779685892,-0.0003793193027,-0.0004779415613,-0.000573544763,-0.0006657579797,-0.0007543310057,-0.0008389379946,-0.0009193728911,-0.0009953536792,-0.001066723256,-0.001133244717,-0.001194810611,-0.001251229201,-0.001302440767,-0.001348303747,-0.001388804056,-0.001423845184,-0.001453461125, -0.00147759926,-0.001496336306,-0.001509663416,  -0.0015176991,-0.001520470367,-0.001518137287,-0.001510761096,-0.001498537604,-0.001481559128,-0.001460050466,-0.001434135251,-0.001404062263, -0.00136997737,-0.001332153683,-0.001290753251,-0.001246065833,-0.001198270009,-0.001147667994,-0.001094443956, -0.00103891117,-0.0009812591597,-0.0009218034684,-0.000860735774,-0.0007983671385,-0.0007348905201,-0.0006706095301,-0.0006057086866,-0.0005404875847,-0.0004751155793,-0.0004098799254,-0.0003449405776,-0.0002805684635,-0.0002169036015,-0.0001542016544,-9.258588398e-05,-3.228907735e-05,2.658227459e-05,8.381998487e-05,0.0001393363491,0.0001929452555,0.0002445846621,0.0002940831182,0.0003414095554,0.0003864161845,0.0004290879006,0.0004693034862, 0.000507074059,0.0005422978429,0.0005750068231,0.0006051238743,0.0006326982402,0.0006576735759,0.0006801217096,0.0007000003825,0.0007174059865,0.0007323015016,0.0007448099204,0.0007549129659,0.0007627345622,0.0007682772703,0.0007716801483,0.0007729508798,0.0007722416194,0.0007695715758,0.0007650969783,0.0007588416338,0.0007509771967, 0.000741525495,0.0007306668558,0.0007184147253,0.0007049577544,0.0006903240574,0.0006746749277,0.0006580550689,0.0006406262983,0.0006224177196,0.0006036003469,0.0005842031096,0.0005643815966,0.0005441572284,0.0005236924044,0.0005029995227,0.0004822277697,0.0004613797355,0.0004405922955,0.0004198956594,0.0003993697464,0.0003790676128,0.0003590719425,0.0003393812513,0.0003201127402,0.0003012612578,0.0002829008445,0.0002650196548,0.0002477022354,0.0002309367992,0.0002147752093, 0.000199207745,0.0001842603815, 0.000169979714,0.0001563065161,0.0001433146244,0.0001310256048,0.0001192694908,0.0001082692761,9.787874296e-05,8.815328329e-05,7.897933392e-05,7.047156396e-05,6.253210449e-05,5.523092841e-05,4.844334762e-05,4.223604992e-05,3.650138751e-05,3.131332778e-05,2.656286961e-05,7.313481183e-05};
float32_t fir_RPM_states[NUM_TAPS] = {0};
float32_t fir_RPM = 0.0f;

void inv_init(void)
{
    FRESULT res;                                 /* FatFs function common result code */
	  // uint8_t wtext[] = "This is STM32 working with FatFs\n"; /* File write buffer */
    pid_controller_current_Ia.limit = voltage_limit;
    pid_controller_current_Id.limit = voltage_limit;
    pid_controller_current_Iq.limit = voltage_limit;
    pid_controller_current_OCP.limit = voltage_limit;
    // uint8_t wlooptext[] = "This is STM32 working with FatFs in main loop\n"; /* File write buffer */
    // log_buf[0][0].LGSTATE = 0;
    for (size_t i = 0; i < 1024; i++)
    {
        Mot_Conv[i] = (int16_t)10*((float)(1650-(3300*i/1024))/Mot_Curr/3.795-1000/3.795);
        // if (Mot_Conv[i] < 0)
        // {
        //   Mot_Conv[i] = 0;
        // }
    }

    // arm_biquad_cascade_df1_init_f32(&biquad_RPM_filter, NUM_SECTIONS, biquad_RPM_coeffs, biquad_RPM_states);
    arm_fir_init_f32(&fir_RPM_filter, NUM_TAPS, fir_RPM_coeffs, fir_RPM_states, 1);

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
    calibrateOffsets(current_offset,DMA_ADC1_arr);
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
    setPhaseVoltage(25,0,_electricalAngle(M_PI*1.5f,pole_pairs),TIM1,0,0,0);
    for (size_t i = 0; i < 2000; i++)
    {
        Get_Encoder_Angle(DMA_ADC2_arr,&angle_now);
        HAL_Delay(10);
        SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr,sizeof(DMA_ADC2_arr));
        if (i >= 1500)
        {
            angle_integrate += angle_now;
        }
    }
    
    // uint16_t read_raw=read(&hspi1, SPI1_CSn_GPIO_Port,SPI1_CSn_Pin,AS5048A_ANGLE);
    float raw_angle;
    SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr,sizeof(DMA_ADC2_arr));
    Get_Encoder_Angle(DMA_ADC2_arr,&raw_angle);
    raw_angle = angle_integrate/500.0f;
    zero_electric_angle=_electricalAngle(raw_angle,pole_pairs);
    setPhaseVoltage(0,0,_electricalAngle(M_PI*1.5f,pole_pairs),TIM1,0,0,0);
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
    #endif
    
    Config_Fdcan1();

    // while (!got_date)
    // {
    //     HAL_Delay(10);
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
  
}

void inv_loop(void)
{
    // continue;
    volatile int sd_now = __HAL_TIM_GET_COUNTER(&htim2);
    FRESULT res;
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
        uint32_t byteswritten;
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim1)
  {
    if(!run)
    {
      run = 1;
      return;
    }
    run = 0;
    HAL_GPIO_TogglePin(LED_TIM_GPIO_Port,LED_TIM_Pin);
    uint16_t ADC1_arr[4] = {0};
    uint16_t ADC2_arr[4] = {0};
    uint16_t ADC3_arr[6] = {0};
    // Read ADC values from DMA buffers
    SCB_InvalidateDCache_by_Addr(DMA_ADC1_arr,sizeof(DMA_ADC1_arr));
    for (size_t i = 0; i < 4; i++)
    {
      ADC1_arr[i] = DMA_ADC1_arr[i];
    }    
    SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr,sizeof(DMA_ADC2_arr));
    for (size_t i = 0; i < 4; i++)
    {
      ADC2_arr[i] = DMA_ADC2_arr[i];
    }    
    SCB_InvalidateDCache_by_Addr(DMA_ADC3_arr,sizeof(DMA_ADC3_arr));
    for (size_t i = 0; i < 6; i++)
    {
      ADC3_arr[i] = DMA_ADC3_arr[i];
    }    
    if(inverter_state == STATE_RUNNING)
    {      
      float delta = percent_torque_requested - last_percent;
      delta = _constrain(delta,-max_ramp,max_ramp);
      last_percent = last_percent + delta;
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
    }
    else
    {
      last_percent = 0.0f;
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
    }
    if(inverter_state == STATE_READY|| inverter_state == STATE_ERROR)
    {
      PID_reset(&pid_controller_current_Id);
      PID_reset(&pid_controller_current_Iq);
      PID_reset(&pid_controller_current_OCP);
      PID_reset(&pid_controller_current_Ia);
      // pid_controller_current_Iq.output_prev = dir*filtered_RPM/electrical_constant;
    }
    
    #ifdef TIMING
    uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim5);     
    #endif

    indexLED++;
    indexHeartbeat++;
    indexStatus++;
    CAN_Timer++;
    

    Get_Encoder_Angle(ADC2_arr,&angle_now);
    int8_t enc_err = 0;
    if(angle_now != angle_now)
    {
      enc_err = 1;
    }
    for (size_t i = 0; i < 4; i++)
    {
      if(ADC2_arr[i] < ENC_UV)
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

    angle_now = _normalizeAngle(angle_now);

    #ifdef OPEN_LOOP_SPEED
    angle_now = _normalizeAngle(last_angle + open_loop_rpm_var/ 60.0f / freq * 2 * M_PI * 4);
    last_angle = angle_now;
    // setPhaseVoltage(Iq_controller_output, Id_controller_output, _electricalAngle(angle_now, pole_pairs),TIM1);    
    #endif
    
    voltage_power_supply = (float)ADC3_arr[0]*DCVPLSB;
    voltage_limit = voltage_power_supply;
    // float filtered_angle = LowPassFilter_operator(angle_now,&filter);
    
    float angular_vel = 0.0f;
    zero_cross += cal_angular_vel(angle_now,&angular_vel);
    float raw_RPM = (float)dir*angular_vel/4/2/M_PI*60;
    filtered_RPM = LowPassFilter_operator((float)dir*angular_vel/4/2/M_PI*60,&filter_RPM);
    // arm_biquad_cascade_df1_f32(&biquad_RPM_filter, &raw_RPM, &biquad_RPM,1);
    arm_fir_f32(&fir_RPM_filter, &raw_RPM, &fir_RPM, 1);
    // float target_torque = max_torque*last_percent;
    float max_derate = _constrain((float)DERATE_END/DERATE_START-(float)abs(filtered_RPM)/DERATE_START,0.0f,1.0f);
    // last_percent = _constrain(last_percent,-max_derate,max_derate);
    float target_Iq = max_current*last_percent;
    float filtered_Iabc[3] = {0.0f};
    float current_phase_dc[3] = {0.0f};
    uint8_t soft_oc_detected = 0;
    soft_oc_sum -= soft_oc_buf[soft_oc_index];
    for (int i=0;i<3;i++){
       	current_phase[i] =(float) (ADC1_arr[i]-current_offset[i])*ACAPLSB;
        //OCP
        filtered_Iabc[i] = LowPassFilter_operator(current_phase[i],&filter_current_Iabc[i]);
        current_phase_dc[i] = LowPassFilter_operator(current_phase[i],&filter_crrent_DC_Iabc[i]);
        if (filtered_Iabc[i] > ACAOCP||filtered_Iabc[i] < -ACAOCP)
        {
          soft_oc_detected = 1;
        }
        // current_phase[i] = filtered_Iabc[i];
    }
    soft_oc_buf[soft_oc_index] = soft_oc_detected;
    soft_oc_sum += soft_oc_buf[soft_oc_index];
    if (soft_oc_sum > SOFT_OC_TIME/2)
    {
      if (inverter_state == STATE_RUNNING)
      {
        Enter_ERROR_State(ERROR_INSTANT_OC);
      }    
    }
    soft_oc_index++;
    if(soft_oc_index == SOFT_OC_TIME)
    {
      soft_oc_index = 0;
    }

    pid_controller_current_Ia.limit = voltage_limit;
    pid_controller_current_Id.limit = voltage_limit;
    pid_controller_current_Iq.limit = voltage_limit;
    pid_controller_current_OCP.limit = voltage_limit;
    pid_controller_current_Iabc[0].limit = voltage_limit;
    pid_controller_current_Iabc[1].limit = voltage_limit;
    pid_controller_current_Iabc[2].limit = voltage_limit;
    
    float Id,Iq;
    cal_Idq(current_phase, _electricalAngle(angle_now, pole_pairs), &Id, &Iq);
    filtered_Iq=LowPassFilter_operator(Iq,&filter_current_Iq);
    filtered_Id=LowPassFilter_operator(Id,&filter_current_Id);
    // filtered_Iq = Iq;
    // filtered_Id = Id;
    Ia = sqrt(filtered_Id*filtered_Id+filtered_Iq*filtered_Iq);

    #ifndef SIXSTEP
    Iq_controller_output=PID_operator(target_Iq-filtered_Iq,&pid_controller_current_Iq);
    Id_controller_output=PID_operator(-filtered_Id,&pid_controller_current_Id);
    float Iabc_controller_output[3] = {0.0f};
    for (size_t i = 0; i < 3; i++)
    {
      Iabc_controller_output[i] = PID_operator(-current_phase_dc[i],&pid_controller_current_Iabc[i]);
    }
    if(abs(filtered_RPM) > 1000 && enable_dc_control == 0)
    {
      enable_dc_control = 1;
    }
    else if (abs(filtered_RPM) < 750 && enable_dc_control == 1)
    {
      enable_dc_control = 0;
    }    
    if(!enable_dc_control)
    {
      for (size_t i = 0; i < 3; i++)
      {
        Iabc_controller_output[i] = 0.0f;
        PID_reset(&pid_controller_current_Iabc[i]);
      } 
    }
    
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
    setPhaseVoltage(Iq_controller_output, Id_controller_output, _electricalAngle(angle_now, pole_pairs),TIM1,-Iabc_controller_output[0],-Iabc_controller_output[1],-Iabc_controller_output[2]);
    #endif

    #ifdef SIXSTEP
    Ia_controller_output=PID_operator(target_Iq-Ia,&pid_controller_current_Ia);
    setSixStepPhaseVoltage(-Ia_controller_output,_electricalAngle(angle_now,pole_pairs),TIM1);
    #endif

    if (indexLED == freq/2)
    {
    	if(inverter_state == STATE_READY)
      {
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
      }
      else if(inverter_state == STATE_ERROR)
      {
        HAL_GPIO_TogglePin(LED_ERR_GPIO_Port,LED_ERR_Pin);
      }
      
    	// HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
      // UART_TX_Send(&huart1,"ping");
    	indexLED=0;
    }

    report_DCV = (uint16_t) roundf(voltage_power_supply*100);
    int16_t report_DCA = (int16_t) roundf((float)(ADC1_arr[3]-current_offset[3])*DCAPLSB*100);
    if (indexHeartbeat == freq/10)
    {
      CAN_Send_Temp(ADC3_arr);
      CAN_Send_State(report_DCV,report_DCA);
      CAN_Send_Heartbeat();
      CAN_Send_Perameter();
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
    if(ADC3_arr[0]*DCVPLSB > 55)
    {
      report_status |= REPORT_STATUS_HV;
    }

    if (indexStatus == freq/100)
    {
      // int_RPM = (int) lroundf(filtered_RPM);
      int16_t report_RPM = (int16_t) roundf(filtered_RPM);
      // int16_t report_RPM = (int16_t) roundf(zero_cross/4*100*60);
      zero_cross = 0.0f;
      int16_t report_torque = (int16_t) roundf(filtered_Iq/max_current*1000);
      // int16_t report_torque = (int16_t) roundf(Ia/max_current*1000);
      // report_torque = LowPassFilter_operator(report_torque,&filter_report_torque);
      CAN_Send_Status(report_status,report_torque,report_RPM);
      indexStatus = 0;
    }

    //Logging
    int16_t IU_100 = (int16_t)roundf(current_phase[0]*100);
    int16_t IV_100 = (int16_t)roundf(current_phase[1]*100);
    int16_t IW_100 = (int16_t)roundf(current_phase[2]*100);
    HAL_RTC_GetDate(&hrtc,&log_date,RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc,&log_time,RTC_FORMAT_BIN);
    if(log_time.Seconds != last_sec)
    {
      log_subsec = 0;
      last_sec = log_time.Seconds;
    }
    log_buf[wr_log_buf_num][wr_log_index%3600].LGHR = log_time.Hours;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGMIN = log_time.Minutes;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGSEC = log_time.Seconds;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGERR = error_state;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGSUBSEC = log_subsec;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGDCV = report_DCV;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGDCA = report_DCA;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGIU = IU_100;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGIV = IV_100;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGIW = IW_100;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGVQ = (int16_t) roundf(Iq_controller_output*10);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGVD = (int16_t) roundf(Id_controller_output*10);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGSINE = ADC2_arr[0]-ADC2_arr[1];
    log_buf[wr_log_buf_num][wr_log_index%3600].LGCOS = ADC2_arr[2]-ADC2_arr[3];
    log_buf[wr_log_buf_num][wr_log_index%3600].LGANG = (uint16_t) roundf(angle_now*100*180/M_PI);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGTCMD = (int16_t) roundf(last_percent*1000);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGSTATE = report_status;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGVU = TIM1->CCR1;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGVV = TIM1->CCR2;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGVW = TIM1->CCR3;
    log_buf[wr_log_buf_num][wr_log_index%3600].LGRPM = (int16_t) roundf(filtered_RPM);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGID = (int16_t) roundf(filtered_Id*100);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGIQ = (int16_t) roundf(filtered_Iq*10);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGZERO = (uint16_t) roundf(zero_electric_angle*100);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGDCIU = (int16_t) roundf(current_phase_dc[0]*100);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGDCIV = (int16_t) roundf(current_phase_dc[1]*100);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGDCIW = (int16_t) roundf(current_phase_dc[2]*100);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGVA = (int16_t) roundf(Iabc_controller_output[0]*10);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGVB = (int16_t) roundf(Iabc_controller_output[1]*10);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGVC = (int16_t) roundf(Iabc_controller_output[2]*10);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGRMSIU = (uint16_t) roundf(RMS_sum[0]/100);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGRMSIV = (uint16_t) roundf(RMS_sum[1]/100);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGRMSIW = (uint16_t) roundf(RMS_sum[2]/100);
    log_buf[wr_log_buf_num][wr_log_index%3600].LGLOGBUF = wr_log_index;

        
    log_subsec++;
    wr_log_index++;

    if(inverter_state == STATE_RUNNING)
    {
      oc_sum -= oc_buf[oc_index];
      if (HAL_GPIO_ReadPin(OC_Fault_GPIO_Port,OC_Fault_Pin) == GPIO_PIN_RESET)
      {
        oc_buf[oc_index] = 1;
      }
      else
      {
        oc_buf[oc_index] = 0;
      }
      oc_sum += oc_buf[oc_index];
      if (oc_sum > HW_OC_TIME/2)
      {
        Enter_ERROR_State(ERROR_HW_OC);
      }
    }
    oc_index++;
    if(oc_index == HW_OC_TIME)
    {
      oc_index = 0;
    }

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
    if(CAN_Timer == freq && inverter_state == STATE_RUNNING)
    {
      enable_hw_oc = 0;
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
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
    if (indexTimer == freq*10)
    {
      max_time = 0;
      min_time = INT16_MAX;
      max_btw = 0;
      indexTimer = 0;
      max_sdwrite = 0;
    }
    indexTimer++;
    #endif

    HAL_GPIO_TogglePin(LED_TIM_GPIO_Port,LED_TIM_Pin);
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
    if (inverter_state == STATE_RUNNING)
    {
      // Enter_ERROR_State(ERROR_HW_OC);
    }     
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
