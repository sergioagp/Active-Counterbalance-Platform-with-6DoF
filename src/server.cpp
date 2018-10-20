#include "server.h"
#include <stm32f4x7_eth_bsp.h>
#include <netconf.h>
#include "touchscreen.h"
#include "acc_gyro.h"
#include "scale.h"
#include "control.h"
//UART//#include "uart.h" // del this
//UART//#include <stdio.h> // del this
#include "platform.h"
#include <stm32f4x7_eth.h>

CServer::CServer(CPlatform* parent) 
	: m_parent(parent)//, m_uart(CUart::getInstance())
	, m_context(sizeof(CStatus), 'C', 400, 240, 3.14f, 6.23f, 0.92f, 0.63f, 0.19f)
	, m_connected(false)
{
	ETH_BSP_Config();
	LwIP_Init();
	
	m_mtx_connection = xSemaphoreCreateMutex();
	m_mq_message = xQueueCreate(10, sizeof(char));
	m_sem_recv_pending = xSemaphoreCreateBinary();
	
	xTaskCreate(tServerOut, "ServerOut", configMINIMAL_STACK_SIZE, this, 2, &m_tserverout);
	xTaskCreate(tServerIn, "ServerIn", configMINIMAL_STACK_SIZE, this, 1, &m_tserverin);
	//UARTxTaskCreate(tUartOut, "UartOut", configMINIMAL_STACK_SIZE, this, 2, NULL);
}

CServer::~CServer() {
	vTaskDelete(m_tserverout);
	vTaskDelete(m_tserverin);
}

bool CServer::setup() {
	err_t err;
	
	m_connection = netconn_new_with_callback(NETCONN_TCP, cbReceive);
	
	if (m_connection != NULL) {
		// bind connection to port 5000
		err = netconn_bind(m_connection, NULL, 5000);
		if (err == ERR_OK) {
			// go into listening mode
			netconn_listen(m_connection);
			return true;
		} else {
			netconn_delete(m_connection);
		}
	}
	return false;
}

err_t CServer::acceptConnection() {
	err_t err;
	err = netconn_accept(m_connection, &m_newconnection);
	return err;
}

void CServer::setConnected(bool mode) {
	if (xSemaphoreTake(m_mtx_connection, portMAX_DELAY) == pdTRUE) {
		m_connected = mode;
		xSemaphoreGive(m_mtx_connection);
	}
}

bool CServer::isConnected() {
	bool ret;
	if (xSemaphoreTake(m_mtx_connection, portMAX_DELAY) == pdTRUE) {
		ret = m_connected;
		xSemaphoreGive(m_mtx_connection);
	}
	return ret;
}

void CServer::tServerOut(void* arg) {
	CServer* server = (CServer*) arg;
	err_t err;
	
	if (server->setup() == false) {
		// delete task if server is not setup
		vTaskDelete(NULL);
	}
	for (;;) {
		err = server->acceptConnection();
		
		if (err == ERR_OK) {
			server->setConnected(true);
			server->startSending();
		}
	}
}

void CServer::tServerIn(void* arg) {
	CServer* server = static_cast<CServer*>(arg);
	
	for (;;) {
		xSemaphoreTake(server->m_sem_recv_pending, portMAX_DELAY);
		server->receive();
	}
}

void CServer::startSending() {
	BaseType_t xStatus;
	char option;
	
	for (;;) {
		xStatus = xQueueReceive(m_mq_message, &option, portMAX_DELAY);
		if (xStatus != pdPASS) {
			// handle error
		}
		
		select(option);
	}
}

void CServer::updateContext() {
	static CTouchScreen* touchscreen = CTouchScreen::getInstance();
	static CAccGyro* accelerometer = CAccGyro::getInstance();
	static CScale* scale = CScale::getInstance();
	static CControl* control = CCounterBalance::getInstance();
	
	coordinates_t coords = touchscreen->getCoordinates();
	slope_t slope = accelerometer->getSlope();
	
	m_context.m_kp = control->getKp();
	m_context.m_ki = control->getKi();
	m_context.m_kd = control->getKd();
	m_context.m_xPos = coords.m_x;
	m_context.m_yPos = coords.m_y;
	m_context.m_xSlope = slope.m_roll;
	m_context.m_ySlope = slope.m_pitch;
	m_context.m_weight = scale->getWeight();
	
	if (m_parent->isBalancing()) {
		m_context.m_mode = 'C';
	} else {
		m_context.m_mode = 'S';
	}
}

err_t CServer::send(const void* data, size_t size) {
	return netconn_write(m_newconnection, data, size, NETCONN_COPY);
}

err_t CServer::receive() {
	netbuf* rxbuf;
	void* data;
	CMessage* message;
	err_t err;
	uint16_t packet_length;
	
	err = netconn_recv(m_newconnection, &rxbuf);
	if (err == ERR_OK) {
		do {
			netbuf_data(rxbuf, &data, &packet_length);
			m_rxbuf.append(data, packet_length);
		} while (netbuf_next(rxbuf) >= 0);
		
		message = reinterpret_cast<CMessage*>(m_rxbuf.getRef());
		if (m_rxbuf.size() >= message->m_size) {
			process(message);
			m_rxbuf.reset();
		}
		netbuf_delete(rxbuf);
	}
	return err;
}

void CServer::cbReceive(netconn* pconn, netconn_evt event, uint16_t len) {
	static CServer* server = CServer::getInstance();
	
	switch (event) {
		case NETCONN_EVT_RCVPLUS:
			if (len > 0) {
				server->listen();
			}
			break;
		default:
			break;
	}
}

void CServer::process(CMessage* message) {
	CPidGain* pid_gain;
	CPos* pos;
	
	switch (message->m_mode) {
		// set new kp gain
		case 'P':
			pid_gain = static_cast<CPidGain*>(message);
			m_parent->setKp(pid_gain->m_gain);
			break;
		// set new ki gain
		case 'I':
			pid_gain = static_cast<CPidGain*>(message);
			m_parent->setKi(pid_gain->m_gain);
			break;
		// set new kd gain
		case 'D':
			pid_gain = static_cast<CPidGain*>(message);
			m_parent->setKd(pid_gain->m_gain);
			break;
		// set new center position
		case 'C':
			pos = static_cast<CPos*>(message);
			m_parent->setCenter(pos->m_x, pos->m_y);
		case 'H':
			// halt the platform
			m_parent->halt();
			break;
		default:
			break;
	}
}

void CServer::listen() {
	xSemaphoreGive(m_sem_recv_pending);
}

err_t CServer::select(char option) {
	static CMessage message('N');
	static CMessage halt('H');
	
	switch (option) {
	case 'C':
		// send context
		updateContext();
		//fwrite(&m_context, 1, sizeof(CStatus), stdout);
		return send(&m_context, sizeof(CStatus));
//		break;
	case 'N':
		// send bye to client
		//fwrite(&message, 1, sizeof(CMessage), stdout);
		return send(&message, sizeof(CMessage));
//		break;
	case 'H':
		// send halt cmd
		//fwrite(&halt, 1, sizeof(CMessage), stdout);
		return send(&halt, sizeof(CMessage));
//		break;
	default:
		break;
	}
	return 0;
}

/*void CServer::tUartOut(void* arg) {
	portBASE_TYPE xStatus;
	CServer* server = static_cast<CServer*>(arg);
	char option;
	
	for (;;) {
		xStatus = xQueueReceive(server->m_mq_message, &option, portMAX_DELAY);
		if (xStatus != pdPASS) {
			// handle error
		}
		
		server->select(option);
	}
}*/

void CServer::sendContext() {
	if (isConnected()) {
		static char option = 'C';
		BaseType_t xStatus;
		
		xStatus = xQueueSendToBack(m_mq_message, &option, 0);
		if (xStatus != pdPASS) {
			// handle error (message could not be sent)
		}
	}
}

void CServer::sendGoodBye() {
	if (isConnected()) {
		static char option = 'N';
		BaseType_t xStatus;
		
		xStatus = xQueueSendToBack(m_mq_message, &option, 0);
		if (xStatus != pdPASS) {
			// handle error (message could not be sent)
		}
	}
}

void CServer::sendHalt() {
	if(isConnected()) {
		static char option = 'H';
		BaseType_t xStatus;
		
		xStatus = xQueueSendToBack(m_mq_message, &option, 0);
		if (xStatus != pdPASS) {
			// handle error (message could not be sent)
		}
	}
}

void CServer::powerDown() {
	ETH_PowerDownCmd(ENABLE);
}
