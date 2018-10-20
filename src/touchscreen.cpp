#include "touchscreen.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "platform.h"

#define ADCRES	4095.0f
#define XSCREEN	800.0f	// mm
#define YSCREEN 400.0f	// mm

extern "C" void EXTI1_IRQHandler(void) {
	static CTouchScreen* touchscreen = CTouchScreen::getInstance();
	static BaseType_t xHigherPriorityTaskWoken;
	static BaseType_t xStatus;
	
	if (EXTI_GetITStatus(EXTI_Line1) == SET) {
		EXTI_ClearITPendingBit(EXTI_Line1);
		
		xStatus = touchscreen->callParent(&xHigherPriorityTaskWoken);
		//touchscreen->start(&xHigherPriorityTaskWoken);
		/* de-initialize exti interrupt */
		touchscreen->changePinsConfig(CHECKOBJ);
		// do the context switch if necessary
		if (xStatus != pdFAIL) {
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
		}
	}
}

extern "C" void TIM7_IRQHandler(void) {
	static CTouchScreen* touchscreen = CTouchScreen::getInstance();
	static BaseType_t xHigherPriorityTaskWoken;
	
	if (TIM_GetITStatus(TIM7, TIM_IT_Update)) {
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		
		TIM_Cmd(TIM7, DISABLE);
		TIM_SetCounter(TIM7, 0U);
		
		touchscreen->read(&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

CTouchScreen::CTouchScreen(CPlatform* parent)
	: m_parent(parent), m_readx(READX), m_ready(READY), m_checkobj(CHECKOBJ), m_touch(TOUCH) 
{
	// important for isr and freertos to work
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	// setup sync objects
	m_mq_coordinates = xQueueCreate(10, sizeof(coordinates_t));
	m_sem_startacquisition = xSemaphoreCreateBinary();
	m_sem_delay = xSemaphoreCreateBinary();
	m_mtx_coordinates = xSemaphoreCreateMutex();
	/* enable peripheral clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// GPIOB RCC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	// GPIOE RCC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	// SYSCFG RCC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);	// ADC2 RCC
	
	setupDelayTimer();
	
	xTaskCreate(tTouchScrOrganizer, "TouchScrOrg", configMINIMAL_STACK_SIZE, this, 6, &m_torganizer);
}
	
CTouchScreen::~CTouchScreen(void) {
	vTaskDelete(m_torganizer);
}

inline void CTouchScreen::read(BaseType_t* xHigherPriorityTaskWoken) {
	xSemaphoreGiveFromISR(m_sem_delay, xHigherPriorityTaskWoken);
	changePinsConfig(CHECKOBJ);
}

coordinates_t CTouchScreen::getCoordinates() {
	coordinates_t coords;
	xSemaphoreTake(m_mtx_coordinates, portMAX_DELAY);
	
	coords = m_coordinates;
	
	xSemaphoreGive(m_mtx_coordinates);
	
	return coords;
}

void CTouchScreen::tTouchScrOrganizer(void* arg) {
	portBASE_TYPE xStatus;
	TickType_t last_exec_time = xTaskGetTickCount();
	CTouchScreen* touchscreen = static_cast<CTouchScreen*>(arg);
	const TickType_t delay = 20 / portTICK_PERIOD_MS;
	
	touchscreen->init();
	for (;;) {
		// take semaphore
		xSemaphoreTake(touchscreen->m_sem_startacquisition, portMAX_DELAY);
		
		if (touchscreen->isObjectOnPlatform()) {
			
			touchscreen->readX();
			touchscreen->readY();
			
			// if platform is in balancing mode, send queue to unblock respective pid task
			if (touchscreen->m_parent->isBalancing()) {
				// send to queue
				xStatus = xQueueSendToBack(touchscreen->m_mq_coordinates, &touchscreen->m_coordinates, 0);
				if (xStatus != pdPASS) {
					// handle error
				}
			}
			// give semaphore back
			xSemaphoreGive(touchscreen->m_sem_startacquisition);
			
			vTaskDelayUntil(&last_exec_time, delay);
		} else { // block task
			touchscreen->m_parent->pause();
		}
	}
}

bool CTouchScreen::isObjectOnPlatform() {
	//static int times = 0;
	
	changePinsConfig(CHECKOBJ);
	// get back to this code when pins are configured right
	vTaskDelay(1 / portTICK_PERIOD_MS);
	//wait();
	
	/*if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET) {
		times = 0;
		return true;
	} else {
		if (times == 0) {
			return false;
		}
		++times;
		if (times > 500) {
			times = 0;
			return false;
		} else {
			return true;
		}
	}*/
	// check if object on platform
	return (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET);
}

void CTouchScreen::readX() {
	static uint16_t last_val = 0;
	uint16_t value;
	
	changePinsConfig(READX);
			
	vTaskDelay(1 / portTICK_PERIOD_MS);
	//wait();
			
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 1, ADC_SampleTime_144Cycles);
	ADC_SoftwareStartConv(ADC2);
			
	while (ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET)
		;
	xSemaphoreTake(m_mtx_coordinates, portMAX_DELAY);
	
	value = ADC_GetConversionValue(ADC2) * (XSCREEN / ADCRES);
	m_coordinates.setX(value * 0.7f + last_val * 0.3f);
	last_val = value;
	
	xSemaphoreGive(m_mtx_coordinates);
}

void CTouchScreen::readY() {
	static uint16_t last_val = 0;
	uint16_t value;
	
	changePinsConfig(READY);
	// get back to this code when pins are configured right
	vTaskDelay(1 / portTICK_PERIOD_MS);
	//wait();
			
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_144Cycles);
	ADC_SoftwareStartConv(ADC2);
			
	while (ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET)
		;
	xSemaphoreTake(m_mtx_coordinates, portMAX_DELAY);
	
	value = ADC_GetConversionValue(ADC2) * (YSCREEN / ADCRES);
	m_coordinates.setY(value * 0.7f + last_val * 0.3f);
	last_val = value;
	
	xSemaphoreGive(m_mtx_coordinates);
}

void CTouchScreen::stop() {
//	xSemaphoreTake(m_mtx_coordinates, portMAX_DELAY);
	changePinsConfig(TOUCH);
}

void CTouchScreen::init() {
	if (isObjectOnPlatform()) {
		m_parent->run();
	} else {
		changePinsConfig(TOUCH);
	}
}

BaseType_t CTouchScreen::callParent(BaseType_t* xHigherPriorityTaskWoken) {
	return m_parent->runFromISR(xHigherPriorityTaskWoken);
}

void CTouchScreen::changePinsConfig(ePinState next_pin_state) {
	/* disable adc and its interrupt
	 * disable exti line and its interrupt
	 */
//	ADC_Cmd(ADC1, DISABLE);
//	ADC_Cmd(ADC2, DISABLE);
//	NVIC_DisableIRQ(ADC_IRQn);
	EXTI_DeInit();
	NVIC_DisableIRQ(EXTI1_IRQn);
	
	switch (next_pin_state) {
		case READX:
			GPIO_Init(GPIOB, &m_readx.m_x1);
			GPIO_Init(GPIOE, &m_readx.m_x2);
			GPIO_Init(GPIOB, &m_readx.m_y1);
			GPIO_Init(GPIOE, &m_readx.m_y2);
			GPIO_SetBits(GPIOB, GPIO_Pin_0);	// x1 high state
			GPIO_ResetBits(GPIOE, GPIO_Pin_7);	// x2 low state
			// GPIO_ResetBits(GPIOB, GPIO_Pin_1);	// y1 low state
			GPIO_ResetBits(GPIOE, GPIO_Pin_8);	// y2 tri state (low)
			ADC_Init(ADC2, &m_readx.m_adc);
//			NVIC_Init(&m_readx.m_nvic_adc);
			ADC_Cmd(ADC2, ENABLE);
			break;
		case READY:
			GPIO_Init(GPIOB, &m_ready.m_x1);
			GPIO_Init(GPIOE, &m_ready.m_x2);
			GPIO_Init(GPIOB, &m_ready.m_y1);
			GPIO_Init(GPIOE, &m_ready.m_y2);
			GPIO_SetBits(GPIOB, GPIO_Pin_1);	// y1 high state
			GPIO_ResetBits(GPIOE, GPIO_Pin_8);	// y2 low state
			// GPIO_ResetBits(GPIOB, GPIO_Pin_0);	// x1 low state
			GPIO_ResetBits(GPIOE, GPIO_Pin_7);	// x2 tri state (low)
			ADC_Init(ADC2, &m_ready.m_adc);
//			NVIC_Init(&m_ready.m_nvic_adc);
			ADC_Cmd(ADC2, ENABLE);
			break;
		case CHECKOBJ:
			GPIO_Init(GPIOB, &m_checkobj.m_x1);
			GPIO_Init(GPIOE, &m_checkobj.m_x2);
			GPIO_Init(GPIOB, &m_checkobj.m_y1);
			GPIO_Init(GPIOE, &m_checkobj.m_y2);
			GPIO_ResetBits(GPIOB, GPIO_Pin_0);	// x1 low state
			GPIO_ResetBits(GPIOE, GPIO_Pin_7);	// x2 tri state (low)
			GPIO_ResetBits(GPIOE, GPIO_Pin_8);	// y2 tri state (low)
			break;
		case TOUCH:
			GPIO_Init(GPIOB, &m_touch.m_x1);
			GPIO_Init(GPIOE, &m_touch.m_x2);
			GPIO_Init(GPIOB, &m_touch.m_y1);
			GPIO_Init(GPIOE, &m_touch.m_y2);
			GPIO_ResetBits(GPIOB, GPIO_Pin_0);	// x1 low state
			//GPIO_SetBits(GPIOB, GPIO_Pin_1);	// y1 high state
			GPIO_ResetBits(GPIOE, GPIO_Pin_7);	// x2 tri state (low)
			GPIO_ResetBits(GPIOE, GPIO_Pin_8);	// y2 tri state (low)
			SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);	// Port B, Pin 1 -> PB1
			EXTI_Init(&m_touch.m_exti_y1);
			NVIC_Init(&m_touch.m_nvic_exti);
			break;
		default:
			break;
	}
}

touchscreengpio_t::touchscreengpio_t(ePinState state) {
	switch (state) {
		case READX:
			initModeX();
			break;
		case READY:
			initModeY();
			break;
		case CHECKOBJ:
			initModeC();
			break;
		case TOUCH:
			initModeT();
			break;
		default:
			break;
	}
}

void touchscreengpio_t::initModeX(void) {
	m_x1.GPIO_Mode = GPIO_Mode_OUT;
	m_x1.GPIO_OType = GPIO_OType_PP;
	m_x1.GPIO_Pin = GPIO_Pin_0;			// PB0
	m_x1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_x1.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_x2.GPIO_Mode = GPIO_Mode_OUT;
	m_x2.GPIO_OType = GPIO_OType_PP;
	m_x2.GPIO_Pin = GPIO_Pin_7;			// PE7
	m_x2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_x2.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_y1.GPIO_Mode = GPIO_Mode_AN;
	// m_y1.GPIO_OType = GPIO_OType_OD;
	m_y1.GPIO_Pin = GPIO_Pin_1;			// PB1
	// m_y1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_y1.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_y2.GPIO_Mode = GPIO_Mode_IN;
	// m_y2.GPIO_OType = GPIO_OType_OD;
	m_y2.GPIO_Pin = GPIO_Pin_8;			// PE8
	m_y2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_y2.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_adc.ADC_ContinuousConvMode = DISABLE;
	m_adc.ADC_DataAlign = ADC_DataAlign_Right;
	m_adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	m_adc.ADC_NbrOfConversion = 1;
	m_adc.ADC_Resolution = ADC_Resolution_12b;
	m_adc.ADC_ScanConvMode = DISABLE;
	
	//m_nvic_adc.NVIC_IRQChannel = ADC_IRQn;
	//m_nvic_adc.NVIC_IRQChannelCmd = ENABLE;
	//m_nvic_adc.NVIC_IRQChannelPreemptionPriority = 0x0F;
	//m_nvic_adc.NVIC_IRQChannelSubPriority = 0x0F;
	
	//ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
}

void touchscreengpio_t::initModeY(void) {
	m_x1.GPIO_Mode = GPIO_Mode_AN;
	// m_x1.GPIO_OType = GPIO_OType_OD;
	m_x1.GPIO_Pin = GPIO_Pin_0;			// PB0
	// m_x1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_x1.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_x2.GPIO_Mode = GPIO_Mode_IN;
	// m_x2.GPIO_OType = GPIO_OType_OD;
	m_x2.GPIO_Pin = GPIO_Pin_7;			// PE7
	m_x2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_x2.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_y1.GPIO_Mode = GPIO_Mode_OUT;
	m_y1.GPIO_OType = GPIO_OType_PP;
	m_y1.GPIO_Pin = GPIO_Pin_1;			// PB1
	m_y1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_y1.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_y2.GPIO_Mode = GPIO_Mode_OUT;
	m_y2.GPIO_OType = GPIO_OType_PP;
	m_y2.GPIO_Pin = GPIO_Pin_8;			// PE8
	m_y2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_y2.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_adc.ADC_ContinuousConvMode = DISABLE;
	m_adc.ADC_DataAlign = ADC_DataAlign_Right;
	m_adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	m_adc.ADC_NbrOfConversion = 1;
	m_adc.ADC_Resolution = ADC_Resolution_12b;
	m_adc.ADC_ScanConvMode = DISABLE;
	
	/* Same as before */
	/* m_nvic.NVIC_IRQChannel = ADC_IRQn;
	m_nvic_adc.NVIC_IRQChannelCmd = ENABLE;
	m_nvic_adc.NVIC_IRQChannelPreemptionPriority = 0x0F;
	m_nvic_adc.NVIC_IRQChannelSubPriority = 0x0F; */
}

void touchscreengpio_t::initModeC(void) {
	m_x1.GPIO_Mode = GPIO_Mode_OUT;
	m_x1.GPIO_OType = GPIO_OType_PP;
	m_x1.GPIO_Pin = GPIO_Pin_0;			// PB0
	m_x1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_x1.GPIO_Speed = GPIO_Speed_100MHz;

	m_x2.GPIO_Mode = GPIO_Mode_IN;
	// m_x2.GPIO_OType = GPIO_OType_OD;
	m_x2.GPIO_Pin = GPIO_Pin_7;			// PE7
	m_x2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_x2.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_y1.GPIO_Mode = GPIO_Mode_IN;
	// m_y1.GPIO_OType = GPIO_OType_PP;
	m_y1.GPIO_Pin = GPIO_Pin_1;			// PB1
	m_y1.GPIO_PuPd = GPIO_PuPd_UP;
	m_y1.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_y2.GPIO_Mode = GPIO_Mode_IN;
	// m_y2.GPIO_OType = GPIO_OType_OD;
	m_y2.GPIO_Pin = GPIO_Pin_8;			// PE8
	m_y2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_y2.GPIO_Speed = GPIO_Speed_100MHz;
}

void touchscreengpio_t::initModeT(void) {
	m_x1.GPIO_Mode = GPIO_Mode_OUT;
	m_x1.GPIO_OType = GPIO_OType_PP;
	m_x1.GPIO_Pin = GPIO_Pin_0;			// PB0
	m_x1.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_x1.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_x2.GPIO_Mode = GPIO_Mode_IN;
	// m_x2.GPIO_OType = GPIO_OType_OD;
	m_x2.GPIO_Pin = GPIO_Pin_7;			// PE7
	m_x2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_x2.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_y1.GPIO_Mode = GPIO_Mode_IN;
	// m_y1.GPIO_OType = GPIO_OType_PP;
	m_y1.GPIO_Pin = GPIO_Pin_1;			// PB1
	m_y1.GPIO_PuPd = GPIO_PuPd_UP;
	m_y1.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_y2.GPIO_Mode = GPIO_Mode_IN;
	// m_y2.GPIO_OType = GPIO_OType_OD;
	m_y2.GPIO_Pin = GPIO_Pin_8;			// PE8
	m_y2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	m_y2.GPIO_Speed = GPIO_Speed_100MHz;
	
	m_exti_y1.EXTI_Line = EXTI_Line1;		// PB1 corresponds to Line 1
	m_exti_y1.EXTI_LineCmd = ENABLE;
	m_exti_y1.EXTI_Mode = EXTI_Mode_Interrupt;
	m_exti_y1.EXTI_Trigger = EXTI_Trigger_Falling;
	
	m_nvic_exti.NVIC_IRQChannel = EXTI1_IRQn;
	m_nvic_exti.NVIC_IRQChannelCmd = ENABLE;
	m_nvic_exti.NVIC_IRQChannelPreemptionPriority = 0x06;
	m_nvic_exti.NVIC_IRQChannelSubPriority = 0x06;
}

void CTouchScreen::setupDelayTimer() {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	
	/* Timer7 runs @ 84MHz */
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 500;					/* 500us period */
	TIM_TimeBaseInitStruct.TIM_Prescaler = 84;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);
	
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_UpdateRequestConfig(TIM7, TIM_UpdateSource_Regular);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_Init(&NVIC_InitStruct);
}

void CTouchScreen::wait() {
	TIM_SetCounter(TIM7, 0U);
	TIM_Cmd(TIM7, ENABLE);
	xSemaphoreTake(m_sem_delay, portMAX_DELAY);
}
