/*
 * File: functions.c
 *
 *  Created on: 1 mars 2017
 *  Last Updated: 1 mars 2017
 *  Version 1.0
 *      Author: Marcus Persson
 *      Email:	marcus.persson@salacia.se
 */
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "functions.h"


void transmit(char output[]){
	for(int i=0;output[i]!='\0';i++){
		USART6->DR = output[i];
		while(!(USART6->SR & USART_SR_TXE)){;}
	}
}
void transmitIR(void){
	char irsend[]={0x00, 0x00, 0x2F, 0x00, 0xD0, 0x06, 0x11, 0xA0, 0x08, 0xD0, 0x01, 0x1A, 0x01, 0x1A, 0x01, 0x1A, 0x03, 0x4E, 0x01, 0x1A, 0x4D, 0xA6, 0x11, 0xA0, 0x04, 0x68, 0x01, 0x1A, 0xBB, 0xCE, 0x22, 0x01, 0x11, 0x22, 0x11, 0x12, 0x22, 0x11, 0x22, 0x21, 0x11, 0x21, 0x11, 0x12, 0x22, 0x12, 0x22, 0x23, 0x82, 0x45};
	for(int i=0;sizeof(irsend)>i;i++){
		USART1->DR = irsend[i];
		while(!(USART1->SR & USART_SR_TXE)){;}
	}
}

int tempCalc(){
	ADC1->SQR3 &= (ADC_SQR3_SQ1 & 0b0000);
	ADC1->CR2 |= ADC_CR2_SWSTART;
	while((ADC1->SR & ADC_SR_EOC)==0){;}
	int temp=(ADC1->DR & ADC_DR_DATA);//calculate exact temperature depeding on thermistor function
	return temp;
}

int lightCalc(int lSenseVal){
	//range 0-4096, 4096 is highest
	int light=1;//calculate exact light setting depending on light sensor function
	return lSenseVal;
}

