/*
 * File: functions.h
 *
 *  Created on: 1 mars 2017
 *  Last updated: 1 mars 2017
 *  Version 1.0
 *      Author: Marcus Persson
 *      Email:	marcus.persson@salacia.se
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

/**********************************************
 * void transmit(char output[])
 * Description: Sends string to the UART to be transmitted
 * Input:	Char array ("string")
 * Output:	None
 *
 **********************************************/
extern void transmit(char output[]);

/*
 * int tempCalc(int thermValue)
 * Description: Calculates the temperature in celcius from ADC input
 * Input: Thermistor ADC value
 * Output: Temperature in C
 */
extern int tempCalc();

/*
 * int lightCalc(int lSenseVal)
 * Description: Calculates the light in X from ADC input
 * Input: Light sensor ADC value
 * Output: Light in X (lumen?)
 */

extern int lightCalc(int lSenseVal);
extern void transmitIR(void);
#endif /* FUNCTIONS_H_ */
