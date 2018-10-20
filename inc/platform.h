#ifndef __PLATFORM_H__
#define __PLATFORM_H__
#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <event_groups.h>

class CTouchScreen;
class CAccGyro;
class CScale;
class CServer;
class CControl;
class CCounterBalance;
class CStabilizer;

class CPlatform {
	bool m_running;
	bool m_balancing;
	// mutex
	SemaphoreHandle_t m_mtx_mode;
	// semaphore
	SemaphoreHandle_t m_sem_sendcontext;
	// event group handle
	EventGroupHandle_t m_evg_platform_events;
	// platform objects
	CTouchScreen* m_touchscreen;
	CScale* m_scale;
	CAccGyro* m_accelerometer;
	CServer* m_server;
	CCounterBalance* m_counterbalancing;
	CStabilizer* m_stabilizing;
	// others
	void startContextCallback();
	void stopContextCallback();
	void setupContextCallback();
	void setupControlPins();
	// set mode
	void setBalancingMode(bool mode);
	// platform control methods
	void privateRun();
	void privateStop();
	void privateHalt();
	void waitForRecover();
	// thread function
	static void tMonitor(void*);
	static void tSwTimer(void*);
	TaskHandle_t m_monitor_handle;
	TaskHandle_t m_swtimer_handle;
	CPlatform();
public:
	static CPlatform* getInstance() {
		static CPlatform instance;
		return &instance;
	}
	~CPlatform();
	void start();
	
	void pause();
	void run();
	void halt();
	BaseType_t runFromISR(BaseType_t*);
	
	BaseType_t toggleRunFromISR(BaseType_t*);
	BaseType_t toggleModeFromISR(BaseType_t*);
	
	bool isRunning();
	bool isBalancing();
	enum {
		RUN = 1 << 0,
		PAUSE = 1 << 1,
		STOP = 1 << 2,
		HALT = 1 << 3,
		RECOVER = 1 << 4,
		TOGGLE_MODE = 1 << 5
	};
	
	void setCenter(uint16_t x, uint16_t y);
	void setKp(float gain);
	void setKi(float gain);
	void setKd(float gain);
};

#endif // __PLATFORM_H__
