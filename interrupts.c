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
#include "interrupts.h"
#include "functions.h"

volatile int temp;
volatile int bright;
volatile int cntr;


void EXTI3_IRQHandler(void){
	transmit("jasper");

}
void EXTI4_IRQHandler(void){
	ADC1->SQR3 &= (ADC_SQR3_SQ1 & 0b0000);
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while((ADC1->SR & ADC_SR_EOC)!=1){;}
	temp=tempCalc((ADC1->DR & ADC_DR_DATA));
	transmit("jasper");

}

void USART6_IRQHandler(void){
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

}
/*void ADC_IRQHandler(void){

}*/

void TIM2_IRQHandler(void){
	ADC1->SQR3 = ADC_SQR3_SQ1_0;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while((ADC1->SR & ADC_SR_EOC)!=1){;}
	bright=lightCalc((ADC1->DR & ADC_DR_DATA));
}

void TIM5_IRQHandler(void){
	cntr++;
	if(cntr>10){
		TIM5->CR1 &=(~TIM_CR1_CEN);
	}
}
