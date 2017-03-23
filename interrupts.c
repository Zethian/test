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
#include <string.h>
#include "functions.h"

volatile int temp;
volatile int bright;
//char irsend[]={0x0000, 0x006c, 0x0022, 0x0002, 0x0155, 0x00ab, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0040, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0015, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x0015, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x0040, 0x0016, 0x05e8, 0x0155, 0x0056, 0x0016, 0x0e3b, '\0'};
char irsend[]={0x00, 0x00, 0x2F, 0x00, 0xD0, 0x06, 0x11, 0xA0, 0x08, 0xD0, 0x01, 0x1A, 0x01, 0x1A, 0x01, 0x1A, 0x03, 0x4E, 0x01, 0x1A, 0x4D, 0xA6, 0x11, 0xA0, 0x04, 0x68, 0x01, 0x1A, 0xBB, 0xCE, 0x22, 0x01, 0x11, 0x22, 0x11, 0x12, 0x22, 0x11, 0x22, 0x21, 0x11, 0x21, 0x11, 0x12, 0x22, 0x12, 0x22, 0x23, 0x82, 0x45};



void EXTI15_10_IRQHandler(void){
	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt
	if(EXTI->PR && EXTI_PR_PR10){ //checks for which button has been pushed should be 10
		temp=tempCalc();
		transmit("jasper");
		transmitIR();
		EXTI->PR |= EXTI_PR_PR10; // should be 10
	}
	else if(EXTI->PR & EXTI_PR_PR15){
		GPIOC->ODR ^= GPIO_ODR_ODR_12; //testing
		EXTI->PR |= EXTI_PR_PR15;
	}

	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt
}

void USART2_IRQHandler(void){
	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt
	char copy;
	int temp;
	while((USART2->SR & USART_SR_RXNE)==0){;} // wait until readable
		copy=USART2->DR & USART_DR_DR; //move data register to copy
		if(copy=='Y'){
			temp=tempCalc();
			if(temp<=1000)
			transmit("It's very cold outside, wear longjohns");
			else if(temp>1000 && temp<=1500)
				transmit("It's a bit chilly, wear a sweater!");
			else if(temp>1500 && temp<=2000)
				transmit("It's room temperature");
			else if(temp>2000 && temp<=3000)
				transmit("WOHO, WE'RE ON HAWAII");
			else if(temp>3000)
				transmit("Welcome to hell");
		transmitIR();
		}
		//strncat(received,&copy,1); // add copy to received
		//if(copy=='\n'){ //if final character read, set flag
		//newchar=1;

		/*while(!(USART2->SR & USART_SR_TXE)){;}
		USART2->DR =received;*/
	/*if(USART2->SR & USART_SR_TXE){
		USART2->DR = *received;
	}*/
	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt

}
void USART1_IRQHandler(void){
	GPIOC->ODR ^= GPIO_ODR_ODR_11; //blinks whenever being used in interrupt
	char copy;
		while((USART1->SR & USART_SR_RXNE)==0){;} // wait until readable
			copy=USART1->DR & USART_DR_DR; //move data register to copy
			copy=tempCalc(copy);
			//strncat(received,&copy,1); // add copy to received
			//if(copy=='\n'){ //if final character read, set flag
			//newchar=1;
			//}
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


