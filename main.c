#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART#
#include "RTE_Device.h"

#define PERIOD 255

void pwmSetPulse(uint16_t pulseTime)
{
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = pulseTime;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM4, &TIM_OCInitStruct);
}

void timerInit(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	 
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	TIM_TimeBaseInitStruct.TIM_Period = PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 999;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM4, &TIM_OCInitStruct);
	TIM_Cmd(TIM4, ENABLE);
}


void USARTInit(void)
{
	GPIO_InitTypeDef port;
	USART_InitTypeDef usart;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	port.GPIO_Speed = GPIO_Speed_50MHz;
	
	port.GPIO_Mode = GPIO_Mode_AF_PP;
	port.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA , &port);
	
	port.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	port.GPIO_Pin= GPIO_Pin_10;
	GPIO_Init(GPIOA , &port);
	
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_BaudRate = 9600;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART1, &usart);
	USART1->CR1|=USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
	USART_ClearFlag (USART1, USART_FLAG_CTS | USART_FLAG_LBD | USART_FLAG_TC | USART_FLAG_RXNE);
	
	USART_Cmd(USART1, ENABLE);
}

/**
 * Обработка USART оповещений.
 * Имеем 2 случая
 * 1) Если принятое слово меньше или равно периоду - ставим новое время импульса
 * отправляем по USART 0 и выходим из прерывания
 * 2) В противном случае отправляем еденицу и тоже выходим из прерывания
 * */
void USART1_IRQHandler(void)
{
  	while((USART1->SR & USART_SR_RXNE)==0);

  	uint16_t pwmPulseTime = USART1->DR;
	
	while((USART1->SR & USART_SR_TXE)==0);
	
	if (pwmPulseTime > PERIOD) 
	{
		USART_SendData(USART1, 1u);
		return;
	}		
	// Так как у нас обратная логика на время импульса, вычитаем из периода
	// время спада - получаем время импульса.
	pwmSetPulse(PERIOD - pwmPulseTime);
	USART_SendData(USART1, 0u);
}

int main()
{
	timerInit();
	USARTInit();
	
	while(1) 
	{
//		if (pwdValue == 0) ticker = 1;
//		else if (pwdValue == PERIOD) ticker = 0;
//		
//		for (uint16_t i = 0; i < 6500U; i++);

//		if (!ticker) pwdValue--;
//		else pwdValue++;
//		pwmSetPulse(pwdValue);
	}
}