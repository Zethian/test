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
	initTimer();
	initPWM();
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
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	/*GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;
	GPIOA->MODER &= ~GPIO_MODER_MODER15;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR15_1;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PA;
	EXTI->IMR |= EXTI_IMR_MR15;
	EXTI->RTSR |= EXTI_RTSR_TR15;*/

	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;
	GPIOC->MODER &= ~GPIO_MODER_MODER10;
	GPIOC->PUPDR |= GPIO_PUPDR_PUPDR10_1;
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PC;
	EXTI->IMR |= EXTI_IMR_MR10; // unmask EXTI15_10 interrupt
	EXTI->RTSR |= EXTI_RTSR_TR10; //rising edge

	NVIC_SetPriority(EXTI15_10_IRQn, 6);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	/*NVIC_SetPriority(EXTI4_IRQn, 6);
	NVIC_EnableIRQ(EXTI4_IRQn);*/
}

void initLED(void){
	GPIOC->MODER |= GPIO_MODER_MODER11_0;
	GPIOC->MODER |= GPIO_MODER_MODER12_0;
}

void initADC(void){
	GPIOA->MODER |= GPIO_MODER_MODER1|GPIO_MODER_MODER0; //sets pins to analog
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enable ADC clock
	//ADC1->CR1 |= ADC_CR1_EOCIE; // enable end-of-conversion interrupt

	ADC1->CR2 |= ADC_CR2_ADON | ADC_CR2_EOCS; //enable ADC,end of regular conversion
	ADC1->CR1 &= (~ADC_CR1_RES); // 12-bit mode
	//ADC1->SMPR2 |= ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 |ADC_SMPR2_SMP0 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP5; //set sampling time to 480 cycles on all used ADC channels
	//ADC->CCR |=ADC_CCR_ADCPRE_0; // set clock prescaler to /8 to slow down ADC sampling time
	ADC1->SQR1 &=(~ADC_SQR1_L); // only one conversion should happen
	NVIC_SetPriority(ADC_IRQn, 2);
	NVIC_EnableIRQ(ADC_IRQn);

}

void initTimer(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->DIER |= TIM_DIER_CC1IE; //enables interrupt
	TIM5->CCMR1 |=TIM_CCMR1_OC1PE; //preload register to protect inbetween interrupts
	TIM5->PSC = 59999; //59999+1 = 0.6ms
	TIM5->CNT = 1; //counter start value
	TIM5->ARR = 100000;//100000; //reload value
	TIM5->CCR1 = 0; //10000
	TIM5->CR1 |= TIM_CR1_CEN|TIM_CR1_DIR; //sets as downcounting and enables clock

	NVIC_SetPriority(TIM5_IRQn, 3);
	NVIC_EnableIRQ( TIM5_IRQn);
}

void initPWM(void){

	/* TIM2 PWM enable */

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	GPIOB->MODER |= GPIO_MODER_MODER10_1; //enable alternate function on PB10
	GPIOB->AFR[1] |= (1<<8); //sets pin as TIM2_CH3

	TIM2->CCMR2 |= TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2; // set to PWM mode 1
	TIM2->CCMR2 |=TIM_CCMR2_OC3PE; //preload register
	TIM2->CR1 |= TIM_CR1_ARPE |TIM_CR1_CMS; //auto-reload preload register and center-aligned
	TIM2->EGR |= TIM_EGR_UG;
	TIM2->CCER |= TIM_CCER_CC3E;
	TIM2->PSC = 9999; //9999+1 = 0.1ms
	//TIM2->CNT = 1; //counter start value
	TIM2->ARR = 10000; //reload value
	TIM2->CCR3 = 8000; //sets cycle for CH3
	TIM2->CR1 |= TIM_CR1_CEN; //enables counter

	/* TIM3 PWM Enable */

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	GPIOC->MODER |= GPIO_MODER_MODER8_1|GPIO_MODER_MODER9_1; //sets pins PC8-9 as alternate functions
	GPIOC->AFR[1] |= (1<<1)|(1<<5); //sets pins as TIM3_CH3 and CH4

	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2 |TIM_CCMR2_OC4M_1|TIM_CCMR2_OC4M_2; // set to PWM mode 1
	TIM3->CCMR2 |=TIM_CCMR2_OC3PE|TIM_CCMR2_OC4PE; //preload register
	TIM3->CR1 |= TIM_CR1_ARPE |TIM_CR1_CMS; //auto-reload preload register and center-aligned
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CCER |= TIM_CCER_CC3E|TIM_CCER_CC4E;
	TIM3->PSC = 9999; //9999+1 = 0.1ms
	//TIM3->CNT = 1; //counter start value
	TIM3->ARR = 10000; //reload value
	TIM3->CCR3 = 1000; //sets cycle for CH3
	TIM3->CCR4 = 500; //sets cycle for CH4

	/* TIM1 PWM Enable */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	GPIOB->MODER |= GPIO_MODER_MODER13_1|GPIO_MODER_MODER14_1|GPIO_MODER_MODER15_1; //sets pins PB13-15 as alternate function
	GPIOB->AFR[1] |= (1<<20)|(1<<24)|(1<<28); //sets pins as TIM1_CH1N,CH2N,CH3N

	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2|TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2;
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2;
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE|TIM_CCMR1_OC2PE;
	TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
	TIM1->CR1 |= TIM_CR1_ARPE|TIM_CR1_CMS;
	TIM1->EGR |= TIM_EGR_UG;
	TIM1->CCER |= TIM_CCER_CC1NE| TIM_CCER_CC2NE | TIM_CCER_CC3NE;
	TIM1->BDTR |= TIM_BDTR_MOE|TIM_BDTR_OSSR;
	TIM1->PSC =9999;
	//TIM1->CNT =1;
	TIM1->ARR = 10000;
	TIM1->CCR1 = 6000;
	TIM1->CCR2 = 4000;
	TIM1->CCR3 = 2000;


	/* TURN IT UP */
	TIM3->CR1 |= TIM_CR1_CEN; //sets as downcounting and enables clock
	TIM1->CR1 |= TIM_CR1_CEN;
}

void initI2S(void){
	//PB3 och 5
	GPIOB->MODER |= GPIO_MODER_MODER3_1|GPIO_MODER_MODER5_1;
	GPIOB->AFR[0] |=(1<<14)|(1<<12)|(1<<20)|(1<<22);
	SPI1->I2SCFGR |=SPI_I2SCFGR_I2SMOD|SPI_I2SCFGR_DATLEN_1;
	SPI1->I2SCFGR |=SPI_I2SCFGR_I2SE;
}

void startup(void){
	//GPIOC->ODR |= GPIO_ODR_ODR_11;

	GPIOC->ODR |= GPIO_ODR_ODR_12;
}


