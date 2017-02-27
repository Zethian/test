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

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void initPORT(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
}
void initUART(void){
	// ENABLE UART receiver
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //enable LED2 output and USART pins
	GPIOC->MODER |= GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1;

	  __disable_irq();
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
	  __enable_irq();
}
void initBUTTONS(void){
	__disable_irq();
	GPIOC->MODER &= ~GPIO_MODER_MODER10;
	GPIOA->MODER &= ~GPIO_MODER_MODER15;
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PC;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PA;

	NVIC_SetPriority(EXTI3_IRQn, 3);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_SetPriority(EXTI4_IRQn, 3);
	NVIC_EnableIRQ(EXTI4_IRQn);
	__enable_irq();
}
void initLED(void){
	GPIOC->MODER &= ~GPIO_MODER_MODER11;
	GPIOC->MODER &= ~GPIO_MODER_MODER12;
}
void transmit(char output[]){
	for(int i=0;output[i]!='\0';i++){
		USART6->DR = output[i];
		while(!(USART6->SR & USART_SR_TXE)){;}
	}

}
void startup(void){
	GPIOC->ODR |= GPIO_ODR_ODR_11;

	GPIOC->ODR |= GPIO_ODR_ODR_12;
}

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	initPORT();
	initUART();
	initBUTTONS();

	startup();

  /**
  *  IMPORTANT NOTE!
  *  The symbol VECT_TAB_SRAM needs to be defined when building the project
  *  if code has been located to RAM and interrupts are used. 
  *  Otherwise the interrupt table located in flash will be used.
  *  See also the <system_*.c> file and how the SystemInit() function updates 
  *  SCB->VTOR register.  
  *  E.g.  SCB->VTOR = 0x20000000;  
  */

  /* TODO - Add your application code here */

  /* Infinite loop */
  while (1)
  {
  }
}

void EXTI3_IRQHandler(void){
	transmit("jasper");

}
void EXTI4_IRQHandler(void){
	transmit("jasper");

}

void USART2_IRQHandler(void){
	char copy;
	while((USART2->SR & USART_SR_RXNE)==0){;} // wait until readable
		copy=USART2->DR & USART_DR_DR; //move data register to copy
		strncat(received,&copy,1); // add copy to received
		if(copy=='\n'){ //if final character read, set flag
		newchar=1;
		}
		/*while(!(USART2->SR & USART_SR_TXE)){;}
		USART2->DR =received;*/
	/*if(USART2->SR & USART_SR_TXE){
		USART2->DR = *received;
	}*/

}
