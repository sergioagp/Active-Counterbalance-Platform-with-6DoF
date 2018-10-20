#ifndef __MESSAGE_H__
#define __MESSAGE_H__
#pragma once

class CMessage {
public:
	CMessage() {}
    CMessage(char mode, uint32_t size = sizeof(CMessage))
        : m_size(size), m_mode(mode) {}
    int32_t m_size;
    char m_mode;
};

class CPidGain : public CMessage {
public:
    CPidGain(float gain = 0.0f, char mode = 'P', uint32_t size = sizeof(CPidGain))
        : CMessage(mode, size), m_gain(gain) {}
    float m_gain;
};

class CPos : public CMessage {
public:
    CPos(uint32_t x = 0, uint32_t y = 0, char mode = 'C',
         uint32_t size = sizeof(CPos))
        : CMessage(mode, size), m_x(x), m_y(y) {}
    uint32_t m_x;
    uint32_t m_y;
};

class CStatus : public CMessage {
public:
    uint16_t m_xPos;
    uint16_t m_yPos;
    float m_xSlope;
    float m_ySlope;
    float m_kp;
    float m_ki;
    float m_kd;
	float m_weight;
	CStatus() {}
	CStatus(int32_t nbytes, char mode, uint16_t xPos, uint16_t yPos,
			  float xSlope, float ySlope, float kp, float ki, float kd)
		: CMessage(mode, nbytes), m_xPos(xPos), m_yPos(yPos),
		  m_xSlope(xSlope), m_ySlope(ySlope), m_kp(kp), m_ki(ki), m_kd(kd)
	{}
};

#endif // __MESSAGE_H__
