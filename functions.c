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

int tempCalc(int thermValue){
	int temp=1;//calculate exact temperature depeding on thermistor function
	return thermValue;
}

int lightCalc(int lSenseVal){
	//range 0-4096, 4096 is highest
	int light=1;//calculate exact light setting depending on light sensor function
	return lSenseVal;
}
void transmitIR(void){
	//something something Send IR
}
