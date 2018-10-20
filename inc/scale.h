#ifndef __SCALE_H__
#define __SCALE_H__
#pragma once

// header for integer types conventions
#include <cstdint>
// FreeRTOS headers
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
// std::notrow
#include <new>

#define LIMIT_W		0xFFFF

class CPlatform;

class CScale {
private:
	// parent class
	CPlatform* m_parent;
	// mutex holder
	SemaphoreHandle_t m_mtx_weight;
	// message queue object
	QueueHandle_t m_mq_weight;
	// weight storing variables
	std::uint16_t m_digitalweight;
	float m_fweight;
	// weight conversion method
	float convertWeight(std::uint16_t);
	// private constructor -> singleton
	CScale(CPlatform* parent);
	CScale(const CScale&) {}
	CScale& operator=(const CScale& rhs) { return *this; }
	// peripheral related methods
	void setupTimer(void);
	void setupADC(void);
	void setupGPIO(void);
	/* task handle */
	TaskHandle_t m_tweightmeasure;
	/* task function */
	static void tWeightMeasure(void*);
public:
	~CScale(void);
	static CScale* getInstance(CPlatform* parent = NULL) {
		static CScale instance(parent);
		return &instance;
	}
	float getWeight(void);
	inline void sendWeight(const std::uint16_t&, BaseType_t*) __attribute__((always_inline));
	void start(void);
	void stop(void);
};

#endif // __LOADCELL_H__
