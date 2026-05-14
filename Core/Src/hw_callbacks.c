/*
 * hw_callbacks.c
 *
 * Miscellaneous HAL callbacks (EXTI, UART, etc.)
 * Centralizes hardware interrupt callbacks that don't belong
 * to a specific application module.
 *
 * Note: CAN callbacks    → can_app.c
 *       Timer callback   → main.c (FOC control loop)
 *       IRQ Handlers     → stm32h7xx_it.c (CubeMX generated)
 */

#include "main.h"
#include "inverter_state.h"
#include "fatfs.h"
#include "logger.h"

/* ================================================================
 *  Extern declarations
 * ================================================================ */
extern int  isSent;       // UART TX busy flag

/* ================================================================
 *  GPIO EXTI Callback
 * ================================================================ */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
  // GATE_Fault (PD3)
  if (GPIO_PIN == GPIO_PIN_3)
  {
    Enter_ERROR_State(ERROR_GATE);
  }
  // OC_Fault (PD6)
  if (GPIO_PIN == GPIO_PIN_6)
  {
    if (inverter_state == STATE_RUNNING)
    {
      // Enter_ERROR_State(ERROR_HW_OC);
    }
  }
  // LV_Fault (PG5)
  if (GPIO_PIN == GPIO_PIN_5)
  {
    // f_write(&MyFile,write_buffer,sizeof(write_buffer),&written);
    f_sync(&MyFile);
    while (1)
    {
      /* code */
    }
  }
}

/* ================================================================
 *  UART TX Complete Callback
 * ================================================================ */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  isSent = 1;
}