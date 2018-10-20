#include <stm32f4xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stm32f4x7_eth.h>
#include <stm32f4x7_eth_bsp.h>
// #include <lwip/sys.h>

extern xSemaphoreHandle s_xSemaphore;
extern xSemaphoreHandle ETH_link_xSemaphore;

/**
  * @brief  This function handles External line 10 interrupt request.
  * @param  None
  * @retval None
  */
/* void EXTI15_10_IRQHandler(void) CHANGED THIS!!
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  if(EXTI_GetITStatus(ETH_LINK_EXTI_LINE) != RESET)
  {*/
  /* Give the semaphore to wakeup LwIP task */
 /* xSemaphoreGiveFromISR( ETH_link_xSemaphore, &xHigherPriorityTaskWoken ); 
  }*/
   /* Clear interrupt pending bit */
 //  EXTI_ClearITPendingBit(ETH_LINK_EXTI_LINE);
  
    /* Switch tasks if necessary. */	
 /* if( xHigherPriorityTaskWoken != pdFALSE )
  {
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
} CHANGED THIS!!!
*/
/**
  * @brief  This function handles ethernet DMA interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* Frame received */
  if ( ETH_GetDMAFlagStatus(ETH_DMA_FLAG_R) == SET) 
  {
    /* Give the semaphore to wakeup LwIP task */
    xSemaphoreGiveFromISR( s_xSemaphore, &xHigherPriorityTaskWoken );
  }

  /* Clear the interrupt flags. */
  /* Clear the Eth DMA Rx IT pending bits */
  ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
  ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);

  /* Switch tasks if necessary. */	
  if( xHigherPriorityTaskWoken != pdFALSE )
  {
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
}
