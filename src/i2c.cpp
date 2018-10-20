#include "i2c.h"
#include <stm32f4xx.h>
#include <cstdint>

CI2C::CI2C(std::uint8_t addr): address(addr)
{
	configRCC();
	configGPIOB();
	configI2C();
}

void CI2C::configRCC(void)
{
	/* --------------------------- System Clocks Configuration -----------------*/
    /* I2C1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
 
    /* GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* DMA1 clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
}

void CI2C::configGPIOB(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
 
	/*-------------------------- GPIO Configuration ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
}

void CI2C::configI2C(void) {
	I2C_InitTypeDef  I2C_InitStructure;
	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	/* Duty cycle 2 because 42Mhz(PCLK1) is mutiple of 1.2Mhz 
	We need mutiples of 10Mhz for 16/9 DC and mutiples of 1.2Mhz for 2 DC
	in order to get 400khz*/
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; 
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_ClockSpeed = I2C_CLOCK;
	
	I2C_Init(I2C1, &I2C_InitStructure);

	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

bool CI2C::start(I2C_direction direction) {
	std::uint16_t timeout = I2C_TIMEOUT_MAX;
	
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0) return false;
	}
	timeout = I2C_TIMEOUT_MAX;
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2C1, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0) return false;
	}
	timeout = I2C_TIMEOUT_MAX;
	// Send slave Address for write 
	I2C_Send7bitAddress(I2C1, address, direction);
	  
	/* wait for I2C1 EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
			/* If the timeout delay is exeeded, exit with error code */
			if ((timeout--) == 0) return false;
		}
		//timeout = I2C_TIMEOUT_MAX;
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
			/* If the timeout delay is exeeded, exit with error code */
			if ((timeout--) == 0) return false;
		}
		//timeout = I2C_TIMEOUT_MAX;
	}
	return true;
}

bool CI2C::write(std::uint8_t data) {
	std::uint16_t timeout = I2C_TIMEOUT_MAX;
	
	I2C_SendData(I2C1, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0) return false;
	}
	return true;
}
bool CI2C::write(std::uint8_t reg, std::uint8_t data) {
	
	if(!start(TransmitterMode)) {
		return false;
	}
	
	if(!write(reg)) {
		return false;
	}
	
	if(!write(data)) {
		return false;
	}
	stop();
	
	return true;
}


std::uint8_t CI2C::read_ack(void){
	std::uint16_t timeout = I2C_TIMEOUT_MAX;
	
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) ) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0) return false;
	}
	// read data from I2C data register and return data byte
	std::uint8_t data = I2C_ReceiveData(I2C1);
	return data;
}


std::uint8_t CI2C::read_nack(void){
	std::uint16_t timeout = I2C_TIMEOUT_MAX;
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) ) {
		/* If the timeout delay is exeeded, exit with error code */
		if ((timeout--) == 0) return false;
	}
	
	// read data from I2C data register and return data byte
	std::uint8_t data = I2C_ReceiveData(I2C1);
	return data;
}

std::uint8_t CI2C::read(std::uint8_t reg) {
	if(!start(TransmitterMode)) {
		return false;
	}
	if(!write(reg)) {
		return false;
	}
	stop();
	if(!start(ReceiverMode)) {
		return false;
	}
	std::uint8_t received_data = read_nack();
	stop();
	return received_data;
}

bool CI2C::read(std::uint8_t reg, std::uint8_t* data, std::size_t size) {
	if(!start(TransmitterMode)) {
		return false;
	}
	if(!write(reg)) {
		return false;
	}
	stop();
	if(!start(ReceiverMode)) {
		return false;
	}
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	for(int i=0; i<(size-1); i++)
		data[i] = (std::uint8_t) read_ack();

	data[size-1] = (std::uint8_t)read_nack();
	return true;
}

void CI2C::stop(void) {
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2C1, ENABLE);
}
