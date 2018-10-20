#ifndef __UART_H__
#define __UART_H__
#pragma once

class CUart {
private:
	const unsigned int m_baudrate;
	void setup();
	CUart(unsigned int baudrate = 115200);
	// forbid this ctor and operator implementation
	CUart(const CUart&) : m_baudrate(115200) {}
	CUart& operator=(const CUart& rhs) { return *this; }
public:
	static CUart* getInstance() {
		static CUart instance;
		return &instance;
	}
	bool readyRead();
};

#endif // __UART_H__
