/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gsm.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Timer.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

#include <stdint.h>
#include <string.h>

#include "gsm.h"

#define CTRLZ "\032"
#define CR "\015"
#define LF "\012"

#define CR_C '\015'
#define LF_C '\012'

// constrant string macro for lazy people not wanting to count the length
#define CSTR(str) str, sizeof(str)-1

void pcReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
	if (count != UART_ERROR && count > 0) {
		// Write the data to the GSM
		UART_writePolling(uartGSM, buf, count);
		// Write the data to the PC too
		UART_writePolling(uartPC, buf, count);
	}
}

#define GSM_MAX_RES_LEN 256
volatile char gsmRes[GSM_MAX_RES_LEN] = { 0 };
volatile unsigned gsmResLen = 0;
// True if the result was truncated in the response buffer
volatile unsigned resTrunc = 0;
enum GSM_RES_STATES
{
	GSM_WAITING_FOR_START_CR,
	GSM_WAITING_FOR_START_LF,
	GSM_WAITING_FOR_END_CR,
	GSM_WAITING_FOR_END_LF
};
volatile enum GSM_RES_STATES gsmResponseState = GSM_WAITING_FOR_START_CR;

inline int gsmResponseReady() {
	return gsmResponseState == GSM_WAITING_FOR_START_CR && gsmResLen > 0;
}

void clearGSMResponse() {
	gsmResLen = 0;
	resTrunc = 0;
}

#define GSM_READ_TIMEOUT_MS 5000
Timer_Handle gsmReadTimer;
volatile int gsmReadTimeout = 0;
void gsmReadTimeoutIsr(UArg arg) {
	if (gsmReadTimeout < GSM_READ_TIMEOUT_MS)
		gsmReadTimeout++;
}

inline int gsmReadTimedout() {
	return gsmReadTimeout >= GSM_READ_TIMEOUT_MS;
}

inline void gsmReadTimeoutReset() {
	gsmReadTimeout = 0;
}

// res MUST have length = GSM_MAX_RES_LEN
void waitForGSMResponse(char *res) {
	char input;

	Timer_start(gsmReadTimer);
	while (!gsmResponseReady()) {
		if (gsmReadTimedout()) {
			Timer_stop(gsmReadTimer);
			gsmReadTimeoutReset();
			clearGSMResponse();
			if (res != NULL)
				res[0] = '\0';

			UART_writePolling(uartPC, CSTR("GSM read timeout\r\n"));
			break;
		}
		UART_read(uartGSM, &input, 1);
	}
	Timer_stop(gsmReadTimer);

	if (res != NULL)
		strcpy(res, (const char *)gsmRes);

	clearGSMResponse();
}

void parseGSMResponse(void *buf, size_t count) {
	int processed = 0;
	while (processed < count) {
		char cur = ((char *)buf)[processed];
		if (gsmResponseState == GSM_WAITING_FOR_START_CR) {
			if (cur == CR_C)
				gsmResponseState = GSM_WAITING_FOR_START_LF;
		} else if (gsmResponseState == GSM_WAITING_FOR_START_LF) {
			if (cur == LF_C) {
				gsmResponseState = GSM_WAITING_FOR_END_CR;
				// reset the response
				clearGSMResponse();
			}
			else if (cur != CR_C) {
				gsmResponseState = GSM_WAITING_FOR_START_CR;
			}
		} else if (gsmResponseState == GSM_WAITING_FOR_END_CR) {
			if (cur == CR_C) {
				gsmResponseState = GSM_WAITING_FOR_END_LF;
			} else {
				// copy
				if (gsmResLen == GSM_MAX_RES_LEN - 1) {
					resTrunc = 1;
					break;
				}
				gsmRes[gsmResLen] = cur;
				gsmResLen++;
			}
		} else if (gsmResponseState == GSM_WAITING_FOR_END_LF) {
			if (cur == LF_C) {
				gsmResponseState = GSM_WAITING_FOR_START_CR;
				gsmRes[gsmResLen] = '\0';
			} else {
				// copy
				if (gsmResLen == GSM_MAX_RES_LEN - 1) {
					resTrunc = 1;
					break;
				}
				gsmRes[gsmResLen] = cur;
				gsmResLen++;

				if (cur != CR_C)
					gsmResponseState = GSM_WAITING_FOR_END_CR;
			}
		}
		processed++;
	}
}

void gsmReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
	if (count != UART_ERROR && count > 0) {
		// Write the data to the PC
		UART_writePolling(uartPC, buf, count);
		parseGSMResponse(buf, count);
	}
}

void setupUART() {
	UART_Params uartPCParams;
	UART_Params uartGSMParams;

	// Create a UART for PC
	System_printf("Initializing PC UART\n");
	//System_flush();
	UART_Params_init(&uartPCParams);
	uartPCParams.readMode = UART_MODE_CALLBACK;
	uartPCParams.readCallback = &pcReadUARTCallback;
	uartPCParams.writeDataMode = UART_DATA_BINARY;
	uartPCParams.readDataMode = UART_DATA_BINARY;
	uartPCParams.baudRate = 9600;
	uartPC = UART_open(Board_UART0, &uartPCParams);

	if (uartPC == NULL) {
		System_abort("Error opening the UART");
	}

	// Create a UART for GSM
	System_printf("Initializing GSM UART\n");
	//System_flush();
	UART_Params_init(&uartGSMParams);
	uartGSMParams.readMode = UART_MODE_CALLBACK;
	uartGSMParams.readCallback = &gsmReadUARTCallback;
	uartGSMParams.writeDataMode = UART_DATA_BINARY;
	uartGSMParams.readDataMode = UART_DATA_BINARY;
	// TODO: reconfigure later, with different baud
	uartGSMParams.baudRate = 9600;
	uartGSM = UART_open(Board_UART1, &uartGSMParams);

	if (uartGSM == NULL) {
		System_abort("Error opening the GSM UART");
	}
}

// send a commant to the GSM module
void cmdGSM(char *cmd, unsigned cmdlen) {
	UART_writePolling(uartPC, cmd, cmdlen);
	UART_writePolling(uartPC, "\r\n", 2);
	UART_writePolling(uartGSM, cmd, cmdlen);
	UART_writePolling(uartGSM, CR, 1);
}

#define EAT_OK 1
#define NO_EAT_OK 0
// lengths don't include the null terminator
// Returns the strcmp result of the received response vs the expected response
int cmdVerifyResponse(char *cmd, unsigned cmdlen, char *expect, unsigned explen, unsigned eatOK) {
	char res[GSM_MAX_RES_LEN] = {0};
	cmdGSM(cmd, cmdlen);
	waitForGSMResponse(res);
	// TODO: this need to be more complex. If there's an error, probably won't get an OK
	// and this will loop forever
	if (eatOK)
		waitForGSMResponse(NULL);

	return strcmp(res, expect);
}


#define CMD_GSM_VERIFY(cmd, expect, eatOK) cmdVerifyResponse(CSTR(cmd), CSTR(expect), eatOK)

// cmd, expect, and error_print must be const strings
#define CMD_GSM_RET_ON_FAILURE(cmd, expect, eatOK, error_print) \
	if (CMD_GSM_VERIFY(cmd, expect, eatOK)) { \
		UART_writePolling(uartPC, CSTR(error_print)); \
		return -1; \
	}

int setupGSM() {
	// TODO: check for +SIMD: 11 and +SIMD: 4, but must have a timeout
	UART_writePolling(uartPC, CSTR("Initializing GSM Connection\r\n"));

	UART_writePolling(uartPC, CSTR("Verifying GPRS attachment\r\n"));
	if (CMD_GSM_VERIFY("AT+CGATT?", "+CGATT: 1", EAT_OK)) {
		// Cell not attached
		UART_writePolling(uartPC, CSTR("Not attached, attaching GPRS now\r\n"));
		CMD_GSM_RET_ON_FAILURE("AT+CGATT=1", "OK", NO_EAT_OK, "Failed to attach GPRS\r\n");
	}

	UART_writePolling(uartPC, CSTR("Verifying network connection\r\n"));
	CMD_GSM_RET_ON_FAILURE("AT+CREG?", "+CREG: 0,1", EAT_OK, "Not registered with network\r\n");

	UART_writePolling(uartPC, CSTR("Setting PDP context\r\n"));
	CMD_GSM_RET_ON_FAILURE("AT+CGDCONT=1,\"IP\",\"breadnet\"", "OK", NO_EAT_OK, "Failed to set PDP context\r\n");
	CMD_GSM_RET_ON_FAILURE("AT+CGACT=1,1", "OK", NO_EAT_OK, "Failed to activate PDP context\r\n");

	UART_writePolling(uartPC, CSTR("GSM Setup Finished\r\n"));

	return 0;
}

int setupTimer() {
	Timer_Params timerParams;
	Error_Block eb;

	Error_init(&eb);
	Timer_Params_init(&timerParams);

	// 1ms timeout
	timerParams.startMode = Timer_StartMode_USER;
	timerParams.period = 1000;
	timerParams.periodType = Timer_PeriodType_MICROSECS;
	timerParams.arg = 1;

	gsmReadTimer = Timer_create(Timer_ANY, gsmReadTimeoutIsr, &timerParams, &eb);

	return gsmReadTimer == NULL;
}

#define DATALEN "3"
#define DATA "GET"
int requestGET() {
	UART_writePolling(uartPC, CSTR("Connecting to Google\r\n"));
	CMD_GSM_RET_ON_FAILURE("AT+SDATACONF=1,\"TCP\",\"www.google.com\",80", "OK", NO_EAT_OK, "TCP configuration failed\r\n");
	CMD_GSM_RET_ON_FAILURE("AT+SDATASTART=1,1", "OK", NO_EAT_OK, "TCP connect failed\r\n");
//	// TODO: Check the socket AT+SDATASTATUS=1
	CMD_GSM_RET_ON_FAILURE("AT+SDATATSEND=1," DATALEN, ">", NO_EAT_OK, "Request to GSM failed\r\n");
	CMD_GSM_RET_ON_FAILURE(DATA CTRLZ, "OK", NO_EAT_OK, "Entering data failed\r\n");

	return 0;
}

void gsmTask(UArg arg0, UArg arg1) {
	char input;

	//System_printf("test");
	//System_flush();

	int timerFail = setupTimer();
	setupUART();
	if (timerFail)
		UART_writePolling(uartPC, CSTR("Failed to create timer\r\n"));
	if (setupGSM())
		UART_writePolling(uartPC, CSTR("Failed to setup GSM\r\n"));
	if (requestGET())
		UART_writePolling(uartPC, CSTR("GET attempt failed\r\n"));

	while (1) {
		// Read from all the UARTS
		UART_read(uartPC, &input, 1);
		if (input == 's') {
			if (setupGSM())
				UART_writePolling(uartPC, CSTR("Failed to setup GSM\r\n"));
		} else if (input == 'g') {
			if (requestGET())
				UART_writePolling(uartPC, CSTR("GET attempt failed\r\n"));
		}
		UART_read(uartGSM, &input, 1);
	}
}
