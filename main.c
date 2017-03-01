/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 7.0.1   2017-02-22

The MIT License (MIT)
Copyright (c) 2009-2016 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************
*/

/* Includes */
#include "stm32f4xx.h"
#include "init.h"
#include "functions.h"

/* Private macro */
/* Private variables */
volatile unsigned int thermistor;
volatile unsigned int brightness;
/* Private function prototypes */
/* Private functions */






/* TODO
 * IR reciever transmission
 * MIC_CLK, MIC_D
 * USB
 * SWDIO
 * SWCLOCK
 * write logic tree on recieved data N/A
 * set Ax as ADC
 *
 */


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	initAll();
  /* Infinite loop */
  while (1) {;}
}

void EXTI3_IRQHandler(void){
	transmit("jasper");

}
void EXTI4_IRQHandler(void){
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
void ADC_IRQHandler(void){
	thermistor=ADC1->DR & ADC_DR_DATA;

}