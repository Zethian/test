/*
 * File: interrupts.c
 *
 *  Created on: 2 mars 2017
 *  Last Updated: 2 mars 2017
 *      Author: Marcus Persson
 *      Email: marcus.persson@salacia.se
 */

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "functions.h"

volatile int temp;
volatile int bright;



void EXTI15_10_IRQHandler(void){
	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt
	if(EXTI->PR && EXTI_PR_PR10){ //checks for which button has been pushed
		ADC1->SQR3 &= (ADC_SQR3_SQ1 & 0b0000);
		ADC1->CR2 |= ADC_CR2_SWSTART;
		while((ADC1->SR & ADC_SR_EOC)==0){;}
		temp=tempCalc((ADC1->DR & ADC_DR_DATA));
		transmit("jasper");
		EXTI->PR |= EXTI_PR_PR10;
	}
	else if(EXTI->PR & EXTI_PR_PR15){
		GPIOC->ODR ^= GPIO_ODR_ODR_12; //testing
		EXTI->PR |= EXTI_PR_PR15;
	}

	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt
}

void USART6_IRQHandler(void){
	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt
	char copy;
	while((USART2->SR & USART_SR_RXNE)==0){;} // wait until readable
		copy=USART2->DR & USART_DR_DR; //move data register to copy
		//strncat(received,&copy,1); // add copy to received
		if(copy=='\n'){ //if final character read, set flag
		//newchar=1;
		}
		/*while(!(USART2->SR & USART_SR_TXE)){;}
		USART2->DR =received;*/
	/*if(USART2->SR & USART_SR_TXE){
		USART2->DR = *received;
	}*/
	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt

}


void TIM5_IRQHandler(void){
	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt
	ADC1->SQR3 = ADC_SQR3_SQ1_0;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while((ADC1->SR & ADC_SR_EOC)==0){;}
	bright=lightCalc((ADC1->DR & ADC_DR_DATA));
	TIM5->SR &=~(1<<1);
	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt
}


