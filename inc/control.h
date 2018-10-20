#ifndef __CONTROL_H__
#define __CONTROL_H__
#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "platformact.h"

#include <stm32f4xx.h>                  // Device header
/* Include ARM math */
#include <arm_math.h>

#include "containers.h"

#define SLOPE_MAX PI/6.0f
#define SLOPE_MIN -PI/6.0f

/* Choose PID parameters */
#define PID_PARAM_KP        100.0f            /* Proporcional */
#define PID_PARAM_KI        0.025f        /* Integral */
#define PID_PARAM_KD        25.0f           /* Derivative */

class CTouchScreen;
class CAccGyro;
class CPlatform;

class CControl {
	// parent class
	CPlatform* m_parent;
	CPlatformAct m_motors;
	float m_maxerror;
	float m_maxgain;
	slope_t m_actslope;
	// mutex
	SemaphoreHandle_t m_mtx_kp;
	SemaphoreHandle_t m_mtx_ki;
	SemaphoreHandle_t m_mtx_kd;
	/* ARM PID Instance, float_32 format */
    float m_pid[3];
	
	inline void setconstraint(arm_pid_instance_f32) __attribute__((always_inline));
	inline arm_pid_instance_f32 pid_cast (float pid[3]) __attribute__((always_inline));
	
	virtual void getCurrent(void) = 0;

	virtual float getPitchError(void) = 0;
	virtual float getRollError(void) = 0;
	virtual void getError(float*, float*) = 0;
	
public:
	CControl(float, CPlatform* parent);
	virtual ~CControl() {}

	void setmaxerror(float);
		
	void actPID(float rollerror, float pitcherror);
	void autotune(void);
		
	float getKp(void);
	float getKi(void);
	float getKd(void);
	void setKp(float gain);
	void setKi(float gain);
	void setKd(float gain);
		
	void resetPosition();
};

class CCounterBalance : public CControl {
private:
	coordinates_t m_center, m_coord;
	TaskHandle_t m_tcounterbalance;
	// mutex
	SemaphoreHandle_t m_mtx_center;
	// thread function 
	static void tCounterBalance(void* arg);

	CCounterBalance(CPlatform* parent);

	virtual void getCurrent(void);
	virtual void getError(float*, float*);

	virtual float getPitchError(void);
	virtual float getRollError(void);
public:
	static CCounterBalance* getInstance(CPlatform* parent = NULL) {
		static CCounterBalance instance(parent);
		return &instance;
	}
	virtual ~CCounterBalance(void);
	
	void setCoordCenter(coordinates_t);
};

class CStabilizer : public CControl {
private:
	slope_t m_resetslope, m_slope;
	TaskHandle_t m_tstabilizer;

	static void tStabilizer(void* arg);

	CStabilizer(CPlatform* parent);

	virtual void getCurrent(void);
	virtual void getError(float*, float*);

	virtual float getPitchError(void);
	virtual float getRollError(void);
public:
	static CStabilizer* getInstance(CPlatform* parent = NULL) {
		static CStabilizer instance(parent);
		return &instance;
	}
	virtual ~CStabilizer(void);
};

#endif // __CONTROL_H__
