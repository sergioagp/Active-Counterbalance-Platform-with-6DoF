#include "platform.h"
#include "touchscreen.h"
#include "scale.h"
#include "platformact.h"
#include "server.h"
#include "acc_gyro.h"
#include "control.h"
#include <stm32f4xx.h>
#include "containers.h"

CPlatform::CPlatform() 
	: m_running(false), m_balancing(true),
	  m_touchscreen(CTouchScreen::getInstance(this)),
	  m_scale(CScale::getInstance(this)),
	  m_accelerometer(CAccGyro::getInstance(this)),
	  m_server(CServer::getInstance(this)),
	  m_counterbalancing(CCounterBalance::getInstance(this)),
	  m_stabilizing(CStabilizer::getInstance(this))
{
	m_evg_platform_events = xEventGroupCreate();
	m_mtx_mode = xSemaphoreCreateMutex();
	m_sem_sendcontext = xSemaphoreCreateBinary();
	setupContextCallback();
	setupControlPins();
	
	xTaskCreate(tMonitor, "Monitor", configMINIMAL_STACK_SIZE, this, 8, &m_monitor_handle);
	xTaskCreate(tSwTimer, "SwTimer", configMINIMAL_STACK_SIZE, this, 1, &m_swtimer_handle);
}
		
CPlatform::~CPlatform() {
	vTaskDelete(m_swtimer_handle);
	vTaskDelete(m_monitor_handle);
}

void CPlatform::tMonitor(void* arg) {
	EventBits_t event_bits;
	CPlatform* platform = static_cast<CPlatform*>(arg);
	
	for (;;) {
		event_bits = xEventGroupWaitBits(
					platform->m_evg_platform_events,
					RUN | PAUSE | STOP | HALT | TOGGLE_MODE,
					pdTRUE, pdFALSE, portMAX_DELAY);
		
		// accelerometer noticed an object
		if ((event_bits & RUN) == RUN) {
			platform->privateRun();
		} else if ((event_bits & PAUSE) == PAUSE) {
			platform->privateStop();
		} else if ((event_bits & STOP) == STOP) {
			// user pressed stop button
			platform->privateStop();
			platform->waitForRecover();
		} else if ((event_bits & HALT) == HALT) {
			// user app sent emergency stop cmd
			platform->privateHalt();
			platform->waitForRecover();
		} else if ((event_bits & TOGGLE_MODE) == TOGGLE_MODE) {
			// toggle modes
			platform->setBalancingMode(!platform->isBalancing());
		}
	}
}

void CPlatform::privateRun() {
	m_running = true;
	m_touchscreen->start();
	m_accelerometer->start();
	m_scale->start();
	// init timer to send context
//	startContextCallback();
}
	
void CPlatform::privateStop() {
	m_running = false;
	m_touchscreen->stop();
	m_scale->stop();
	m_accelerometer->stop();
//	stopContextCallback();
	//m_counterbalancing->resetPosition();
//	m_server->sendGoodBye();
}

void CPlatform::privateHalt() {
	m_running = false;
	m_touchscreen->stop();
	m_scale->stop();
	m_accelerometer->stop();
	// stop timer to send context
//	stopContextCallback();
	//m_counterbalancing->resetPosition();
//	m_server->sendHalt();
}

void CPlatform::waitForRecover() {
	xEventGroupWaitBits(m_evg_platform_events, 
						RECOVER, pdTRUE, pdFALSE,
						portMAX_DELAY);
	xEventGroupClearBits(m_evg_platform_events, RUN);
}

void CPlatform::start(void) {
	vTaskStartScheduler();
}

bool CPlatform::isRunning() {
	return m_running;
}

void CPlatform::setBalancingMode(bool mode) {
	if (xSemaphoreTake(m_mtx_mode, portMAX_DELAY) == pdTRUE) {
		m_balancing = mode;
		xSemaphoreGive(m_mtx_mode);
	}
}

bool CPlatform::isBalancing() {
	bool ret;
	if (xSemaphoreTake(m_mtx_mode, portMAX_DELAY) == pdTRUE) {
		ret = m_balancing;
		xSemaphoreGive(m_mtx_mode);
	}
	return ret;
}

void CPlatform::pause() {
	xEventGroupSetBits(m_evg_platform_events, PAUSE);
}

void CPlatform::run() {
	xEventGroupSetBits(m_evg_platform_events, RUN);
}

BaseType_t CPlatform::runFromISR(BaseType_t* xHigherPriorityTaskWoken) {
	return xEventGroupSetBitsFromISR(m_evg_platform_events, RUN, xHigherPriorityTaskWoken);
}

void CPlatform::halt() {
	xEventGroupSetBits(m_evg_platform_events, HALT);
}

BaseType_t CPlatform::toggleRunFromISR(BaseType_t* xHigherPriorityTaskWoken) {
	if (m_running == true) {
		return xEventGroupSetBitsFromISR(m_evg_platform_events, STOP, xHigherPriorityTaskWoken);
	} else {
		return xEventGroupSetBitsFromISR(m_evg_platform_events, RECOVER, xHigherPriorityTaskWoken);
	}
}

BaseType_t CPlatform::toggleModeFromISR(BaseType_t* xHigherPriorityTaskWoken) {
	return xEventGroupSetBitsFromISR(m_evg_platform_events, TOGGLE_MODE, xHigherPriorityTaskWoken);
}

extern "C" void vApplicationIdleHook() {
	static CPlatform* platform = CPlatform::getInstance();
	
	if (platform->isRunning()) {
		// do nothing
	} else {
		// enter stop mode
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
		// after wake-up
		SystemInit();
	}
}

/*extern "C" void TIM2_IRQHandler(void) {
	static CServer* server = CServer::getInstance();
	static BaseType_t xHigherPriorityTaskWoken;
	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		// send context...
		server->sendContextFromISR(&xHigherPriorityTaskWoken);
		
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}*/

void CPlatform::tSwTimer(void* arg) {
	CPlatform* platform = static_cast<CPlatform*>(arg);
	
	for (;;) {
		if (xSemaphoreTake(platform->m_sem_sendcontext, portMAX_DELAY) == pdTRUE) {
			platform->m_server->sendContext();
			xSemaphoreGive(platform->m_sem_sendcontext);
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
	}
}

/*void CPlatform::setupContextCallback() {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	// Timer7 runs @ 84MHz
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 1000000U;					// 1s period
	TIM_TimeBaseInitStruct.TIM_Prescaler = 84;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x08;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
}*/

void CPlatform::startContextCallback() {
	xSemaphoreGive(m_sem_sendcontext);
	//TIM_Cmd(TIM2, ENABLE);
}

void CPlatform::stopContextCallback() {
	xSemaphoreTake(m_sem_sendcontext, portMAX_DELAY);
	//TIM_Cmd(TIM2, DISABLE);
}

void CPlatform::setupControlPins() {
	GPIO_InitTypeDef gpio_is;
	EXTI_InitTypeDef exti_is;
	NVIC_InitTypeDef nvic_is;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	// SYSCFG RCC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
	gpio_is.GPIO_Mode = GPIO_Mode_IN;
	gpio_is.GPIO_OType = GPIO_OType_PP;
	gpio_is.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;			// PE2 and PE3
	gpio_is.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_is.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE, &gpio_is);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	
	exti_is.EXTI_Line = EXTI_Line2;
	exti_is.EXTI_LineCmd = ENABLE;
	exti_is.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_is.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&exti_is);
	
	exti_is.EXTI_Line = EXTI_Line3;
	EXTI_Init(&exti_is);
	
	nvic_is.NVIC_IRQChannel = EXTI2_IRQn;
	nvic_is.NVIC_IRQChannelCmd = ENABLE;
	nvic_is.NVIC_IRQChannelPreemptionPriority = 0x07;
	nvic_is.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&nvic_is);
	
	nvic_is.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&nvic_is);
}

extern "C" void EXTI2_IRQHandler(void) {
	static CPlatform* platform = CPlatform::getInstance();
	static BaseType_t xHigherPriorityTaskWoken;
	static BaseType_t xResult;
	
	if (EXTI_GetITStatus(EXTI_Line2) == SET) {
		EXTI_ClearITPendingBit(EXTI_Line2);
		
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = platform->toggleRunFromISR(&xHigherPriorityTaskWoken);
		if (xResult != pdFAIL) {
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

extern "C" void EXTI3_IRQHandler(void) {
	static CPlatform* platform = CPlatform::getInstance();
	static BaseType_t xHigherPriorityTaskWoken;
	static BaseType_t xResult;
	
	if (EXTI_GetITStatus(EXTI_Line3) == SET) {
		EXTI_ClearITPendingBit(EXTI_Line3);
		
		xHigherPriorityTaskWoken = pdFALSE;
		xResult = platform->toggleModeFromISR(&xHigherPriorityTaskWoken);
		if (xResult != pdFAIL) {
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

void CPlatform::setCenter(uint16_t x, uint16_t y) {
	coordinates_t coords (x, y);
	m_counterbalancing->setCoordCenter(coords);
}

void CPlatform::setKp(float gain) {
	m_counterbalancing->setKp(gain);
	m_stabilizing->setKp(gain);
}

void CPlatform::setKi(float gain) {
	m_counterbalancing->setKi(gain);
	m_stabilizing->setKi(gain);	
}

void CPlatform::setKd(float gain) {
	m_counterbalancing->setKd(gain);
	m_stabilizing->setKd(gain);
}
