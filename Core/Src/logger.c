#include "logger.h"

// Logging Buffers
__attribute__((section("._RAM_Area"))) logger_t log_buf[2][3600];
uint8_t wr_log_buf_num = 0;
uint16_t wr_log_index = 0;
RTC_DateTypeDef log_date;
RTC_TimeTypeDef log_time;
uint8_t last_sec = 0;
uint16_t log_subsec = 0;
static uint32_t prev_new_file = 0;
static uint32_t prev_sd = 0;
uint8_t got_date = 0;
uint8_t last_got_date = 0;
int max_sd_buf = 0;

// SD card
__attribute__((section("._RAM_Area"))) FATFS SDFatFS_RAM;  /* File system object for SD card logical drive */
__attribute__((section("._RAM_Area"))) FIL MyFile;     /* File object */
const char TestFPath[] = {TEST_FILE_PATH};
char TextFPath[80];

/* -----------------------------------------------------------------------
 * Timer: htim2 @ 10 kHz  (Prescaler=24000-1, 240 MHz APB1 timer clock)
 *        1 tick = 100 µs
 * ----------------------------------------------------------------------- */
#define SD_WRITE_INTERVAL       1000U       /* 1 000 ticks = 100 ms   */
#define FILE_ROTATE_INTERVAL    3000000U    /* 3 000 000 ticks = 5 min */
#define SD_WRITE_MAX_RETRIES    3

void set_one(logger_t log_struct[2][2])
{
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            log_struct[i][j].LGHR = 0x1;
            log_struct[i][j].LGMIN = 0x1;
            log_struct[i][j].LGSEC = 0x1;
            log_struct[i][j].LGSUBSEC = 0x1;
            log_struct[i][j].LGDCV = 0x1;
            log_struct[i][j].LGDCA = 0x1;
            log_struct[i][j].LGIU = 0x1;
            log_struct[i][j].LGIV = 0x1;
            log_struct[i][j].LGIW = 0x1;
            log_struct[i][j].LGVQ = 0x1;
            log_struct[i][j].LGVD = 0x1;
            log_struct[i][j].LGSINE = 0x1;
            log_struct[i][j].LGCOS = 0x1;
            log_struct[i][j].LGANG = 0x1;
            log_struct[i][j].LGTCMD = 0x1;
            log_struct[i][j].LGSTATE = 0x1;
            log_struct[i][j].LGVV = 0x1;
            log_struct[i][j].LGVU = 0x1;
            log_struct[i][j].LGVW = 0x1;
            log_struct[i][j].LGRPM = 0x1;
        }
    }
}

void sd_logger_init(void)
{
    volatile FRESULT res;                      /* FatFs function common result code */

    BSP_SD_Init();
    HAL_SD_InitCard(&hsd1);

    uint8_t sdcard_status = HAL_SD_GetCardState(&hsd1);
    if (sdcard_status == HAL_SD_CARD_TRANSFER)
    {
        res = f_mount(&SDFatFS_RAM, (TCHAR const*)SDPath, 1);
        // res = f_mount(&SDFatFS,"0",0);
        if (res != FR_OK)
        {
            Error_Handler();
        }
        else
        {
            #ifdef SDDEBUG
            uint32_t byteswritten;             /* File write/read counts */
            res = f_open(&MyFile,TestFPath,FA_CREATE_ALWAYS | FA_WRITE);
            if (res != FR_OK)
            {
                Error_Handler();
            }
            else
            {
                set_one(log_test);
                f_write(&MyFile, &log_test, sizeof(log_test), (void *)&byteswritten);
                res = f_close(&MyFile);
                if (res != FR_OK)
                {
                    Error_Handler();
                }
            }
            #endif
        }
    }
}

void sd_logger_open_file(void)
{
    volatile FRESULT res;

    // Get DateTime
    HAL_RTC_GetTime(&hrtc, &log_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &log_date, RTC_FORMAT_BIN);

    snprintf(TextFPath, sizeof(TextFPath), FILENAME, (int)log_date.Year+2000, (int)log_date.Month, (int)log_date.Date, (int)log_time.Hours, (int)log_time.Minutes, (int)log_time.Seconds);
    // snprintf(TextFPath, sizeof(TextFPath),"text.bin");
    res = f_open(&MyFile, TextFPath, FA_CREATE_ALWAYS|FA_WRITE);
    if (res == FR_OK)
    {
        f_close(&MyFile);
    }
    f_open(&MyFile, TextFPath, FA_OPEN_APPEND|FA_WRITE);

    prev_sd = __HAL_TIM_GET_COUNTER(&htim2);
    prev_new_file = __HAL_TIM_GET_COUNTER(&htim2);
}

/*
 * reopen_log_file() — close current file, open a new one named by
 *                      the latest RTC timestamp (log_date / log_time).
 */
static FRESULT reopen_log_file(void)
{
    f_close(&MyFile);

    snprintf(TextFPath, sizeof(TextFPath), FILENAME,
             (int)log_date.Year + 2000, (int)log_date.Month,
             (int)log_date.Date,        (int)log_time.Hours,
             (int)log_time.Minutes,     (int)log_time.Seconds);

    /* FA_CREATE_ALWAYS creates (or truncates) file, close immediately,
       then reopen in APPEND mode — ensures file pointer is at end */
    FRESULT res = f_open(&MyFile, TextFPath, FA_CREATE_ALWAYS | FA_WRITE);
    if (res == FR_OK)
    {
        f_close(&MyFile);
    }
    else
    {
        return res;
    }

    res = f_open(&MyFile, TextFPath, FA_OPEN_APPEND | FA_WRITE);
    return res;
}

void sd_logger_run(void)
{
    uint32_t sd_now = __HAL_TIM_GET_COUNTER(&htim2);
    // uint32_t whileTest = sd_now;

    /* ------- 1. File rotation -------- */
    if (sd_now - prev_new_file >= FILE_ROTATE_INTERVAL)
    {
        FRESULT res = reopen_log_file();
        if (res != FR_OK)
        {
            // !!!!! TO-DO: error handling !!!!!
            return;
        }
        prev_new_file = sd_now;
    }

    /* ------- 2. Periodic SD flush -------- */
    if (sd_now - prev_sd < SD_WRITE_INTERVAL)
    {
        return; // early return (not time to write yet!)
    }

    HAL_GPIO_WritePin(LED_SD_GPIO_Port, LED_SD_Pin, GPIO_PIN_SET);

    // If CAN just sent new date, reopen file with correct filename
    if (last_got_date != got_date)
    {
        FRESULT res = reopen_log_file();
        if (res != FR_OK)
        {
            // !!!!! TO-DO: error handling !!!!!
            return;
        }
    }

    // Swap double-buffer (critical section: ISR is writing wr_log_buf_num/wr_log_index)
    __disable_irq();
    uint8_t buf_num_to_sd = wr_log_buf_num;
    uint16_t index_to_sd = wr_log_index;
    wr_log_buf_num ^= 0x1;
    wr_log_index = 0;
    __enable_irq();

    if (max_sd_buf < index_to_sd)
    {
        max_sd_buf = index_to_sd;
    }

    uint32_t byteswritten;
    FRESULT res = f_write(&MyFile, log_buf[buf_num_to_sd],
                          index_to_sd * sizeof(logger_t),
                          (void *)&byteswritten);

    for (int retry = 0; res != FR_OK && retry < SD_WRITE_MAX_RETRIES; retry++)
    {
        if (res != FR_DISK_ERR)
        {
            break;  // Not hardware error, don't retry
        }

        // Try reopening file and write again
        f_open(&MyFile, TextFPath, FA_OPEN_APPEND | FA_WRITE);
        res = f_write(&MyFile, log_buf[buf_num_to_sd],
                      index_to_sd * sizeof(logger_t),
                      (void *)&byteswritten);
    }

    // Only sync on successful write, avoid operating on invalid handle
    if (res == FR_OK)
    {
        f_sync(&MyFile);
    }

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