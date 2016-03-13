/*
 * gsm.h
 *
 *  Created on: Mar 6, 2016
 *      Author: david
 */

#ifndef UARTGSM_GSM_H_
#define UARTGSM_GSM_H_

UART_Handle uartPC = NULL;
UART_Handle uartGSM = NULL;

void gsmTask(UArg arg0, UArg arg1);

#endif /* UARTGSM_GSM_H_ */
