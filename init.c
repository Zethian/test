/*
 * File: init.c
 *
 *  Created on: 27 feb. 2017
 *  Last Updated: 1 mars 2017
 *  Version: 1.0
 *      Author: Marcus Persson
 *      email: 	Marcus.persson@salacia.se
 */
#include "stm32f4xx.h"
#include "init.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

void initAll(void){
	__disable_irq();
	initPORT();
	initUART();
	initBUTTONS();
	initLED();
	initADC();
	initPWM();
	initTimer();
	startup();
	__enable_irq();

}
void initPORT(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
}

void initUART(void){
	// ENABLE UART receiver
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //enable LED2 output and USART pins
	GPIOC->MODER |= GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1;
	  RCC->APB2ENR |= RCC_APB2ENR_USART6EN; //activate USART6 clock
	  GPIOC->AFR[0] |= (7<<27) |(7<<31);

	  USART6->CR1 |= USART_CR1_UE; //Enable UART
	  USART6->CR1 &= ~USART_CR1_M; //set 8bit
	  USART6->CR2 &= ~USART_CR2_STOP; //set 1 stopbit
	  USART6->BRR = (162<<4)|(12<<0); //set 19200bps baud rate
	  USART6->CR1 |= USART_CR1_RE; //enable receiver
	  USART6->CR1 |= USART_CR1_RXNEIE; //enable receiver interrupt

	// ENABLE UART transmitter
	  USART6->CR1 |= USART_CR1_TE;


	  NVIC_SetPriority(USART6_IRQn,1);
	  NVIC_EnableIRQ(USART6_IRQn);
}

void initBUTTONS(void){
	GPIOC->MODER &= ~GPIO_MODER_MODER10;
	GPIOA->MODER &= ~GPIO_MODER_MODER15;
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PC;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PA;

	NVIC_SetPriority(EXTI3_IRQn, 6);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_SetPriority(EXTI4_IRQn, 6);
	NVIC_EnableIRQ(EXTI4_IRQn);
}

void initLED(void){
	GPIOC->MODER |= GPIO_MODER_MODER11_0;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
}

void initADC(void){
	/* måste sätta pins till analog? */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enable ADC clock
	ADC1->CR1 &= (~ADC_CR1_RES); // 12-bit mode
	ADC1->CR1 |= ADC_CR1_EOCIE; // enable end-of-conversion interrupt

	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_EOCS; //enable ADC,end of regular conversion
	//ADC1->SMPR2 |= ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 |ADC_SMPR2_SMP0 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP5; //set sampling time to 480 cycles on all used ADC channels
	ADC->CCR |=ADC_CCR_ADCPRE_0; // set clock prescaler to /8 to slow down ADC sampling time
	ADC1->SQR1 &=(~ADC_SQR1_L); // only one conversion should happen
	NVIC_SetPriority(ADC_IRQn, 2);
	NVIC_EnableIRQ(ADC_IRQn);

}

void initTimer(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->DIER |= TIM_DIER_CC1IE; //enables interrupt
	TIM2->CCMR1 |=TIM_CCMR1_OC1PE; //preload register to protect inbetween interrupts
	TIM2->PSC = 59999; //59999+1 = 0.6ms
	TIM2->CNT = 1; //counter start value
	TIM2->ARR = 100000; //reload value
	TIM2->CCR1 = 0; //10000
	TIM2->CR1 |= TIM_CR1_CEN|TIM_CR1_DIR; //sets as downcounting and enables clock

	NVIC_SetPriority(TIM2_IRQn, 3);
	NVIC_EnableIRQ( TIM2_IRQn);
}

void initPWM(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->DIER |= TIM_DIER_CC1IE; //enables interrupt
	TIM5->CCMR1 |= TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2; // set to PWM mode 1
	TIM5->CCMR1 |=TIM_CCMR1_OC1PE; //preload register
	TIM5->CR1 |= TIM_CR1_ARPE; //auto-reload preload register
	TIM5->EGR |= TIM_EGR_UG;
	TIM5->PSC = 9999; //59999+1 = 0.6ms =
	TIM5->CNT = 1; //counter start value
	TIM5->ARR = 10000; //reload value
	TIM5->CCR1 = 1000; //10000
	TIM5->CR1 |= TIM_CR1_CEN|TIM_CR1_DIR; //sets as downcounting and enables clock
	NVIC_SetPriority(TIM5_IRQn, 5);
	NVIC_EnableIRQ( TIM5_IRQn);
}

void startup(void){
	GPIOC->ODR |= GPIO_ODR_ODR_11;

	GPIOC->ODR |= GPIO_ODR_ODR_12;
}


