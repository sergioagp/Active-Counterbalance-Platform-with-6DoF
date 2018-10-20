#include "servomotor.h"
#include <stm32f4xx.h>

/* ISR */
/*extern "C" void TIM4_IRQHandler(void) {
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		
		
	}
}*/

CServoMotor::CServoMotor(void) {
	setupPeripherals();
}

inline void CServoMotor::setupPeripherals(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	configGpioD();
	configGpioC();
	configTim3();
	configTim4();
}

void CServoMotor::configGpioD(void) {
	GPIO_InitTypeDef gpio_is;
	/*-------------------------- GPIO Configuration ----------------------------*/
	/* GPIOD Configuration:  in output push-pull */
	gpio_is.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio_is.GPIO_Mode = GPIO_Mode_AF;
	gpio_is.GPIO_OType = GPIO_OType_PP;
	gpio_is.GPIO_PuPd = GPIO_PuPd_UP; // weak pull-up resistor for safety during startup
	gpio_is.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &gpio_is);
 
	/* Connect TIM4 pins to AF2 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);// TIM4_CH1 SERVO1
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
}

void CServoMotor::configGpioC(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	/*-------------------------- GPIO Configuration ----------------------------*/
	/* GPIOD Configuration:  in output push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
 
	/* Connect TIM3 pins to AF2 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);// TIM3_CH1 SERVO1
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
}

void CServoMotor::configTim3(void)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//NVIC_InitTypeDef nvic_is;
     
    /* Time base configuration - SystemCoreClock = 168000000/2 for 84 MHz board */
    TIM_TimeBaseStructure.TIM_Prescaler = 84;
    TIM_TimeBaseStructure.TIM_Period = PERIOD_SVMOTOR - 1; //
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

     
    /* TIM PWM1 Mode configuration: Channel */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = DEFPULSE_SVMOTOR; // Servo 90 DEG (central position)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
     
    /* Output Compare PWM1 Mode configuration: Channel1 */
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
     
    /* Output Compare PWM1 Mode configuration: Channel2 */
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/* Output Compare PWM1 Mode configuration: Channel3 */
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/* Output Compare PWM1 Mode configuration: Channel4 */
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
}

void CServoMotor::configTim4(void) {
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	// NVIC_InitTypeDef nvic_is;
     
    /* Time base configuration - SystemCoreClock = 168000000/2 for 84 MHz board */
    TIM_TimeBaseStructure.TIM_Prescaler = 84;
    TIM_TimeBaseStructure.TIM_Period = PERIOD_SVMOTOR - 1; //
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

     
    /* TIM PWM1 Mode configuration: Channel */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = DEFPULSE_SVMOTOR; // Servo 90 DEG (central position)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
     
    /* Output Compare PWM1 Mode configuration: Channel1 ? */
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
     
    /* Output Compare PWM1 Mode configuration: Channel2 ? */
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
}

void CServoMotor::start() {
    /* TIM4 enable counter */
    TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

void CServoMotor::setPosition(servos_t<NSERVOS> servo) {	
	TIM_SetCompare1(TIM3, servo.pulse[0]);
	TIM_SetCompare2(TIM3, servo.pulse[1]);
	TIM_SetCompare3(TIM3, servo.pulse[2]);
	TIM_SetCompare4(TIM3, servo.pulse[3]);
	TIM_SetCompare1(TIM4, servo.pulse[4]);
	TIM_SetCompare2(TIM4, servo.pulse[5]);
}
