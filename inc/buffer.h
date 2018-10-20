#ifndef __BUFFER_H__
#define __BUFFER_H__
#pragma once

#include <cstdint>

class CBuffer {
	int m_index;
	char m_buf[128];
public:
	CBuffer() : m_index(0) {}
	~CBuffer() {}
	void append(void*, std::uint16_t);
	void reset() { m_index = 0; }
	int size() { return m_index; }
	char* getRef() { return m_buf; }
};

#endif // __BUFER_H__
