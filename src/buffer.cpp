#include "buffer.h"
#include <cstring>

void CBuffer::append(void* data, std::uint16_t len) {
	std::memcpy(&m_buf[m_index], data, len);
	m_index += len;
}
