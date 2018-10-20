#include "uart.h"
#include <stm32f4xx.h>
#include <stdio.h>

#define OLEN 128 // size of serial transmission buffer

static uint8_t ostart = 0; // transmission buffer start index
static uint8_t oend = 0; // transmission buffer end index

static volatile char outbuf[OLEN]; // storage for transmission buffer

#define ILEN 128 // size of serial receiving buffer

static uint8_t istart = 0; // receiving buffer start index
static uint8_t iend = 0; // receiving buffer end index

static volatile char inbuf[ILEN]; // storage for receiving buffer

bool sendactive = false, sendfull = false;

CUart::CUart(unsigned int baudrate) 
	: m_baudrate(baudrate) 
{
	setup();
}

// UART ISR
extern "C" void USART2_IRQHandler(void) {
	if (USART_GetITStatus(USART2, USART_IT_RXNE)) {
		char c = USART_ReceiveData(USART2);
		if (istart + ILEN != iend) {
			inbuf[iend++ & (ILEN-1)] = c;
			// put character into buffer
		}
	}
	
	if (USART_GetITStatus(USART2, USART_IT_TXE)) {
		if (ostart != oend) {
			USART_SendData(USART2, outbuf[(ostart++ & (OLEN - 1))]);
			sendfull = false;
		}
		else {
			USART_ClearITPendingBit(USART2, USART_IT_TXE);
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
			sendactive = false;
		}
	}
}

void CUart::setup() {
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_USART2);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVIC_InitStruct);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART2, ENABLE);
}

bool CUart::readyRead() {
	return iend == istart;
}

static void putbuf(char c) {
	if (!sendfull) {
		if (!sendactive) {
			sendactive = true;
			USART_SendData(USART2, c);
			USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
		}
		else {
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
			outbuf[(oend++ & (OLEN-1))] = c;
			// put char to transmission buffer
			if (((oend ^ ostart) & (OLEN-1)) == 0) {
				sendfull = true;
			}
			USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
		}
	}
}

int std::fputc(int ch, FILE* stream) {
	while (sendfull)
		;
	putbuf(ch);
	return ch;
}

int std::fgetc(FILE* stream) {
	char c;
	while (iend == istart)
		;
	c = inbuf[istart++ & (ILEN - 1)];
	return c;
}
