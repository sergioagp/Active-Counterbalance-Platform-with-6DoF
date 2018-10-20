#ifndef __ACC_GYRO_H__
#define __ACC_GYRO_H__
#pragma once

#include <cstdint>
#include <cstdio>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "mpu6050def.h"
#include "i2c.h"
#include "containers.h"

#define BUFFSIZE	20
#define ACCGYRO_PRD_MS 20

class CPlatform;

class CAccGyro {
private:
	// parent class
	CPlatform* m_parent;
	//
	CI2C m_i2c;
	std::uint8_t buffer[BUFFSIZE];
	slope_t m_slope;
	// semaphore to start acquisition
	SemaphoreHandle_t m_sem_startacquisition; 
	// mqueue objects
	QueueHandle_t m_mq_slope;
	QueueHandle_t m_mq_measure;
	// mutex
	SemaphoreHandle_t m_mtx_slope;

	TaskHandle_t m_tslopemeasure;

	CAccGyro(CPlatform* parent);
	void configRCC(void);
	void configDMA(void);
	void configNVIC(void);

	bool setup(void);
	bool check(void);

	static void tSlopeMeasure(void* arg);

public:
	static CAccGyro* getInstance(CPlatform* parent = NULL) {
		static CAccGyro instance(parent);
		return &instance;
	}
	
	void start() { xSemaphoreGive(m_sem_startacquisition); }
	void stop() { xSemaphoreTake(m_sem_startacquisition, portMAX_DELAY); }

	bool readAll();
	
	void setlowpower(void);

	inline bool getMQSlope(slope_t* slope) __attribute__((always_inline)) {
		BaseType_t xStatus;
	
		xStatus = xQueueReceive(m_mq_slope, slope, portMAX_DELAY);
		if (xStatus != pdPASS) { // handle error
			// code here
			return false;
		}
		return true;
	}
	
	slope_t getSlope(void);
	inline void sendRawMeasure(MPU6050_t& mpu6050_data, BaseType_t* xHigherPriorityTaskWoken) __attribute__((always_inline));
	inline MPU6050_t getRawMeasure(void) __attribute__((always_inline));
	slope_t getAccelAngles(MPU6050_t mpu6050_data);
};


#endif // __ACC_GYRO_H__
