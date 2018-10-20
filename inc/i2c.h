#ifndef __I2C_H__
#define __I2C_H__
#pragma once

#include <cstdint>
#include <cstdio>


#define I2C_CLOCK		400000            /*!< Default I2C clock speed */
#define I2C_TIMEOUT_MAX 0xFFFF

enum I2C_direction {
	TransmitterMode = 0,
	ReceiverMode
};

class CI2C {
private:
	std::uint8_t address;

	void configRCC(void);
	void configGPIOB(void);
	void configI2C(void);
public:
	CI2C(std::uint8_t addr);
	bool start(I2C_direction direction);
	bool write(std::uint8_t data);
	bool write(std::uint8_t reg, std::uint8_t data);
	std::uint8_t read_ack(void);
	std::uint8_t read_nack(void);
	std::uint8_t read(std::uint8_t reg);
	bool read(std::uint8_t reg, std::uint8_t* data, std::size_t size);
	void stop(void);

};

#endif //__I2C_H__
