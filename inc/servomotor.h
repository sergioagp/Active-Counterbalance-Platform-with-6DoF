#ifndef __SERVOMOTOR_H__
#define __SERVOMOTOR_H__
#pragma once

#include <cstdint>

#define NSERVOS		6

#define PERIOD_SVMOTOR	10000U 	// us
#define DEG0_SVMOTOR	1000U 	// us
#define DEG90_SVMOTOR	1500U 	// us
#define DEG180_SVMOTOR	2000U 	// us

#define DEFPULSE_SVMOTOR	DEG90_SVMOTOR //us
#define MAXPULSE_SVMOTOR 2000U	//us
#define MINPULSE_SVMOTOR 1000U	//us

template <std::uint8_t NBR>
struct servos_t {
	const std::uint8_t n_servos;
	std::uint16_t pulse[NBR];
	servos_t() : n_servos(NBR) {}
};

class CServoMotor {
private:
	// private ctor (singleton)
	CServoMotor(void);
	// peripheral init methods
	void configGpioD(void);
	void configGpioC(void);
	void configTim3(void);
	void configTim4(void);
	inline void setupPeripherals(void);
public:
	static CServoMotor* getInstance(void) {
		static CServoMotor instance;
		return &instance;
	}
	
	template<typename T>
	static T constrain(T Value, T Min, T Max) {
		return (Value < Min)? Min : (Value > Max)? Max : Value;
	}
	
	void start();
	void setPosition(servos_t<NSERVOS>);
};

#endif // __SERVOMOTOR _H__
