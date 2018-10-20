#ifndef __SERVER_H__
#define __SERVER_H__
#pragma once

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <lwip/api.h>
#include "message.h"
#include "buffer.h"

class CUart;
class CPlatform;

class CServer {
private:
	// parent class
	CPlatform* m_parent;
	//
	//UART//CUart* m_uart;
	// store information to be sent
	CStatus m_context;
	// connected flag & mutex
	bool m_connected;
	SemaphoreHandle_t m_mtx_connection;
	// task handle
	TaskHandle_t m_tserverout;
	TaskHandle_t m_tserverin;
	// semaphore object
	SemaphoreHandle_t m_sem_recv_pending;
	// message queue to send data
	QueueHandle_t m_mq_message;
	// buffer to store rx data
	CBuffer m_rxbuf;
	// netconn objects
	netconn* m_connection, * m_newconnection;
	// private ctor -> singleton
	CServer(CPlatform* parent);
	// forbid this ctor and operator implementation
	CServer(const CServer&) {}
	CServer& operator=(const CServer& rhs) { return *this; }
	// setup server
	bool setup();
	//
	err_t acceptConnection();
	//
	void startSending();
	err_t receive();
	err_t send(const void* data, size_t size);
	// process method
	void process(CMessage*);
	void updateContext();
	// output methods
	err_t select(char);
	// connection methods
	void setConnected(bool mode);
	// thread functions
	static void tServerOut(void*);
	static void tServerIn(void*);
	//UART//static void tUartOut(void*);
	// receive callback function
	static void cbReceive(netconn*, netconn_evt, uint16_t);
public:
	~CServer();
	static CServer* getInstance(CPlatform* parent = NULL) {
		static CServer instance(parent);
		return &instance;
	}
	inline void listen() __attribute__((always_inline));
	inline void sendContextFromISR(BaseType_t* xHigherPriorityTaskWoken) __attribute__((always_inline)) {
		if (isConnected()) {
			static BaseType_t xStatus;
			static char option = 'C';
			xStatus = xQueueSendToBackFromISR(m_mq_message, &option, xHigherPriorityTaskWoken);
			if (xStatus != pdPASS) {
				// handle error (message could not be sent)
			}
		}
	}	
	void sendContext();
	void sendGoodBye();
	void sendHalt();
	bool isConnected();
	void powerDown();
};

#endif // __SERVER_H__
