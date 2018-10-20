#ifndef __PLATFORMACT_H__
#define __PLATFORMACT_H__
#pragma once

#include <cstdint>
#include "servomotor.h"

#include <stm32f4xx.h>                  // Device header
/* Include ARM math */
#include <arm_math.h>

#include "containers.h"

class CPlatformAct {
private:
	/* points of platform's joints */
	point_t m_basejoint[NSERVOS];
	point_t m_platformjoint[NSERVOS];
	point_t m_leglength[NSERVOS];

	slope_t m_rotation;
	point_t m_translation;
	// servo pulse
	CServoMotor* m_servomotor;
	// servo pulse
	servos_t<NSERVOS> m_servo;

	/* angles */
	float m_alpha[NSERVOS];

	inline void setupPeripherals(void);
	// init joints locations
	void initJoints(void);
	void initBetaAngles(void);

	void getRotationMatrix(float[3][3]);

	// servo motors angles
	void calcAlpha(void);
	void calcLengths(void);
	void calcServoPulse();
	// motor positioning
	void setPosition();
	void reset();
public:
	CPlatformAct(void);

	// static inline methods
	static float degToRad(float degree) { return degree * PI / 180.0f; }
	static float radToDeg(float radians) { return radians * 180.0f / PI; }
	
	// method to translation platform
	void setTranslation(const point_t pos);
	// method to rotate platform
	void setRotation(const slope_t&);
	
	void defaultPosition();
};

#endif // __PLATFORMACT _H__
