/*
 * File: init.h
 *
 *  Created on: 27 feb. 2017
 *  Last updated: 1 mars 2017
 *  Version: 1.0
 *      Author: Marcus Persson
 *      Email:	marcus.persson@salacia.se
 */

#ifndef INIT_H_
#define INIT_H_

/**********************************************
 * void initALL(void)
 * Description: Calls all initialization functions
 * Input: 	None
 * Output:	None
 **********************************************/
void initAll(void);

/**********************************************
 * void initPORT()
 * Description: Initializes gpio ports A, B and C on the CPU
 * Input:	None
 * Output:	None
 **********************************************/
void initPORT(void);

/**********************************************
 * void initUART(void)
 * Description: Initializes UART2 with 19200bps baud rate, 8bit, 1 stop bit
 * Input:	None
 * Output:	None
 **********************************************/
void initUART(void);

/**********************************************
 * void initBUTTONS(void)
 * Description: Initializes the input and interrupts for the buttons on the PCB
 * Input:	None
 * Output: None
 **********************************************/
void initBUTTONS(void);

/**********************************************
 * void initLED(void)
 * Description: Sets the specific ports that LEDs are connected to to general output mode
 * Input:	None
 * Output:	None
 **********************************************/
void initLED(void);

/**********************************************
 * void initADC(void)
 * Description: Starts the ADC on ADC channels 0 to 6 and starts ADC interrupts and sets continuous mode
 * Input: 	None
 * Output: 	None
 **********************************************/
void initADC(void);

/**********************************************
 * void startup(void)
 * Description: Performs startup rituals
 * Input:	None
 * Output:	None
 **********************************************/
void startup(void);
void initTimer(void);
void initPWM(void);
void initIR(void);


#endif /* INIT_H_ */
