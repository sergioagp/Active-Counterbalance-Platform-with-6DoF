#ifndef __TOUCHSCREEN_H__
#define __TOUCHSCREEN_H__
#pragma once

#include <stm32f4xx.h>
#include <cstdint>
#include <cstdio>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "containers.h"

class CPlatform;

enum ePinState {
	READX = 1, READY, CHECKOBJ, TOUCH
};

struct touchscreengpio_t {
	GPIO_InitTypeDef m_x1, m_x2;
	GPIO_InitTypeDef m_y1, m_y2;
	ADC_InitTypeDef m_adc;
	NVIC_InitTypeDef m_nvic_exti;//, m_nvic_adc;
	EXTI_InitTypeDef m_exti_y1;
	touchscreengpio_t(ePinState);
private:
	void initModeX(void);
	void initModeY(void);
	void initModeC(void);
	void initModeT(void);
};

class CTouchScreen {
	// parent class
	CPlatform* m_parent;
	// task handle
	TaskHandle_t m_torganizer;
	// message queue object
	QueueHandle_t m_mq_coordinates;
	// semaphore object
	SemaphoreHandle_t m_sem_startacquisition;
	SemaphoreHandle_t m_sem_delay;
	// mutex
	SemaphoreHandle_t m_mtx_coordinates;
	// others
	coordinates_t m_coordinates;
	touchscreengpio_t m_readx, m_ready, m_checkobj, m_touch;
	// private constructor -> singleton
	CTouchScreen(CPlatform* parent);
	// no implementation for copy constructor
	CTouchScreen(const CTouchScreen&);
	// assignment operator overload, does nothing
	CTouchScreen& operator=(const CTouchScreen& rhs) { return *this; }
	// 
	void setupDelayTimer();
	void setupContextTimer();
	void wait();
	void init();
	inline bool isObjectOnPlatform() __attribute__((always_inline));
	inline void readX() __attribute__((always_inline));
	inline void readY() __attribute__((always_inline));
	// freertos task behaviour
	static void tTouchScrOrganizer(void*);
public:
	~CTouchScreen(void);
	static CTouchScreen* getInstance(CPlatform* parent = NULL) {
		static CTouchScreen instance(parent);
		return &instance;
	}
	void changePinsConfig(ePinState);
	
	inline BaseType_t callParent(BaseType_t*) __attribute__((always_inline));
	inline void read(BaseType_t*) __attribute__((always_inline));
	inline bool getMQCoordinates(coordinates_t* coords) __attribute__((always_inline)) {
		BaseType_t xStatus;
	
		xStatus = xQueueReceive(m_mq_coordinates, coords, portMAX_DELAY);
		if (xStatus != pdPASS) { // handle error
			// code here
			return false;
		}
		return true;
	}
	inline void start() {
		xSemaphoreGive(m_sem_startacquisition);
	}
	void stop();
	coordinates_t getCoordinates();
};

#endif // __TOUCHSCREEN_H__
