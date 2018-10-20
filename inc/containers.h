#ifndef __CONTAINERS_H__
#define __CONTAINERS_H__
#pragma once

#include <cstdint>

#ifndef PI
	#define PI	3.14159265358979f
#endif

template<typename T>
static T constrain(T Value, T Min, T Max) {
	return (Value < Min)? Min : (Value > Max)? Max : Value;
}

struct coordinates_t {
	std::uint16_t m_x;
	std::uint16_t m_y;
	
	//coordinates_t(void) : m_x(0), m_y(0) {}
	coordinates_t(std::uint16_t x = 0, std::uint16_t y = 0) : m_x(x), m_y(y) {}
	void setX(std::uint16_t x) { m_x = x; }
	void setY(std::uint16_t y) { m_y = y; }
	coordinates_t operator-(const coordinates_t& coord) {
		coordinates_t coords;
		coords.m_x = this->m_x - coord.m_x;
		coords.m_y = this->m_y - coord.m_y;
		return coords;
	}
};

struct point_t {
	float x;
	float y;
	float z;
	
	point_t(void) : x(0), y(0), z(0) {}
	point_t(float x, float y, float z)  : x(x), y(y), z(z) {}
};

struct slope_t {
	float m_roll;
	float m_pitch;
	float m_yaw;
	slope_t(float r = 0, float p = 0, float y = 0) : m_roll(r), m_pitch(p), m_yaw(y) {}
	slope_t operator-(const slope_t& rhs) {
		slope_t slope;
		slope.m_roll = this->m_roll - rhs.m_roll;
		slope.m_pitch = this->m_pitch - rhs.m_pitch;
		slope.m_yaw = this->m_yaw - rhs.m_yaw;
		return slope;
	}
};

#endif // __CONTAINERS_H__
