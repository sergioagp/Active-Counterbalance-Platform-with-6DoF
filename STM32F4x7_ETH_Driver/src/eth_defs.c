#include "eth_defs.h"
#include <FreeRTOS.h>
#include <task.h>

/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of Ticks to delay.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  vTaskDelay(nCount);
}
