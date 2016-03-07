/*
 * gsm.h
 *
 *  Created on: Mar 6, 2016
 *      Author: david
 */

#ifndef GSM_H_
#define GSM_H_

UART_Handle uartPC;
UART_Handle uartGSM;

void gsmTask(UArg arg0, UArg arg1);

#endif /* GSM_H_ */
