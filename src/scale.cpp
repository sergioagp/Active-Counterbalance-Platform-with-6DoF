#include "scale.h"
#include <stm32f4xx.h>
#include <FreeRTOS.h>
#include <queue.h>
#include "platform.h"

#define MAX 		4095.0f
#define VMAX 		3.0f
#define MAX_WEIGHT	5.0f

// ISR
extern "C" void ADC_IRQHandler(void) {
	static CScale* scale = CScale::getInstance();
	static std::uint16_t adc_val;
	static BaseType_t xHigherPriorityTaskWoken;
	
	if (ADC_GetITStatus(ADC1, ADC_IT_EOC) == SET) {
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		
		adc_val = ADC_GetConversionValue(ADC1);
		scale->sendWeight(adc_val, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

CScale::CScale(CPlatform* parent)
		: m_parent(parent), m_digitalweight(0), 
		  m_fweight(0.0f)
{
	// important for isr and freertos to work
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	m_mtx_weight = xSemaphoreCreateMutex();
	// setup message queue
	m_mq_weight = xQueueCreate(10, sizeof(std::uint16_t));
	// setup peripherals
	setupGPIO();
	setupADC();
	setupTimer();
	// create task
	xTaskCreate(tWeightMeasure, "tWeightMeasure", configMINIMAL_STACK_SIZE, this, 5, &m_tweightmeasure);
}
		  
CScale::~CScale(void) {
	vTaskDelete(m_tweightmeasure);
}
		  
void CScale::start(void) { 
	ADC_Cmd(ADC1, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
}

void CScale::stop(void) { 
	ADC_Cmd(ADC1, DISABLE);
	TIM_Cmd(TIM5, DISABLE);
}

inline void CScale::sendWeight(const std::uint16_t& val, BaseType_t* xHigherPriorityTaskWoken) {
	static BaseType_t xStatus;
	xStatus = xQueueSendToBackFromISR(m_mq_weight, &val, xHigherPriorityTaskWoken);
	if (xStatus != pdPASS) {
		// handle error (message could not be sent)
	}
}

float CScale::convertWeight(std::uint16_t weight) {
	m_digitalweight = weight;
	
	xSemaphoreTake(m_mtx_weight, portMAX_DELAY);
	
	m_fweight = MAX_WEIGHT * (float) m_digitalweight / MAX;
	
	xSemaphoreGive(m_mtx_weight);
	return m_fweight;
}

float CScale::getWeight() {
	float ret = 0.0f;
	
	if (xSemaphoreTake(m_mtx_weight, portMAX_DELAY)) {
		ret = m_fweight;
		xSemaphoreGive(m_mtx_weight);
	}
	return ret;
}

void CScale::tWeightMeasure(void* arg) {
	BaseType_t xStatus;
	std::uint16_t weight;
	CScale* scale = static_cast<CScale*>(arg);
	
	for (;;) {
		xStatus = xQueueReceive(scale->m_mq_weight, &weight, portMAX_DELAY);
		if (xStatus != pdPASS) { // handle error
			// code here
		} else {
			scale->convertWeight(weight);
		}
	}
}

void CScale::setupGPIO(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void CScale::setupADC(void) {
	ADC_InitTypeDef ADC_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;	// cycle through the channels several times
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T5_CC1;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE; // enable multi channels conversion mode
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStruct);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_144Cycles);
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	
	NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x06;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
	
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_Cmd(ADC1, DISABLE);
}

void CScale::setupTimer(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
	/* Timer5 runs @ 84MHz */
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 1000000U;					/* 1s period */
	TIM_TimeBaseInitStruct.TIM_Prescaler = 84;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	/* TIM_OCInitStruct.TIM_OutputNState = TIM_OutputState_Disable; */
	/* only for timer 1 and 8 */
	TIM_OCInitStruct.TIM_Pulse = 1; /* only 1 timer overflow needed to start ADC conversion */
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	/* TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low; */ /* only for timer 1 and 8 */
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset; /* only for timer 1 and 8 */
	/* TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Set; */ /* only for timer 1 and 8 */
	TIM_OC1Init(TIM5, &TIM_OCInitStruct);
	
	TIM_SetAutoreload(TIM5, 1000000U);
	TIM_Cmd(TIM5, DISABLE);
}
