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

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

#include <stdint.h>
#include <string.h>

#include "gsm.h"

// Don't pass in NULL to these
// Should only be used with constant strings
#define UART_WRITE(handle, buf) UART_writePolling(handle, buf, sizeof(buf)-1)
#define UART_WRITELN(handle, buf) UART_writePolling(handle, buf "\r\n", sizeof(buf)+1)

#define CTRLZ "\032"
#define CR "\015"
#define LF "\012"

#define CR_C '\015'
#define LF_C '\012'

void pcReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
	if (count != UART_ERROR && count > 0) {
		// Write the data to the GSM
		UART_writePolling(uartGSM, buf, count);
		// Write the data to the PC too
		UART_writePolling(uartPC, buf, count);
	}
}

void pcWriteUARTCallback(UART_Handle handle, void *buf, size_t count) {
	// Empty!
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
}

// res MUST have length = GSM_MAX_RES_LEN
void waitForGSMResponse(char *res) {
	char input;
	while (!gsmResponseReady())
			UART_read(uartGSM, &input, 1);

	strcpy(res, gsmRes);

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
				gsmResLen = 0;
				resTrunc = 0;
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

void gsmWriteUARTCallback(UART_Handle handle, void *buf, size_t count) {
	// Empty!
}

void setupUART() {
	UART_Params uartPCParams;
	UART_Params uartGSMParams;

	// Create a UART for PC
	System_printf("Initializing PC UART\n");
	//System_flush();
	UART_Params_init(&uartPCParams);
	uartPCParams.readMode = UART_MODE_CALLBACK;
	uartPCParams.writeMode = UART_MODE_CALLBACK;
	uartPCParams.readCallback = &pcReadUARTCallback;
	uartPCParams.writeCallback = &pcWriteUARTCallback;
	uartPCParams.writeDataMode = UART_DATA_BINARY;
	uartPCParams.readDataMode = UART_DATA_BINARY;
	uartPCParams.readReturnMode = UART_RETURN_NEWLINE;
	uartPCParams.readEcho = UART_ECHO_OFF;
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
	uartGSMParams.writeMode = UART_MODE_CALLBACK;
	uartGSMParams.readCallback = &gsmReadUARTCallback;
	uartGSMParams.writeCallback = &gsmWriteUARTCallback;
	uartGSMParams.writeDataMode = UART_DATA_BINARY;
	uartGSMParams.readDataMode = UART_DATA_BINARY;
	uartGSMParams.readReturnMode = UART_RETURN_NEWLINE;
	uartGSMParams.baudRate = 9600;
	uartGSM = UART_open(Board_UART1, &uartGSMParams);

	if (uartGSM == NULL) {
		System_abort("Error opening the GSM UART");
	}
}

// Don't call this directly; instead, use the CMD_GSM_CHECK macro
int expectGSMResponse(const char *expect, unsigned nexp) {
	char input;
	int i = -1;

	// This will fail on the (response, expected) = ("\r\n1234\r\n", "\r\n1234\r\nextra\r\n") pair
	// however, this is an invalid response sequence so I won't bother assuming
	// people using the function are that stupid
	while (i < nexp) {
		// Race condition with the other read?
		// TODO: change this later
		if (UART_read(uartGSM, &input, 1) > 0) {
			i++;
			UART_WRITELN(uartPC, "inc");
		}

		if (i >= 0 && input != expect[i]) {
			UART_WRITELN(uartPC, "ret");
			return -1;
		}
	}

	return 0;
}

#define CMD_GSM(cmd) \
	UART_WRITELN(uartPC, cmd); \
	UART_WRITE(uartGSM, cmd CR);
#define CMD_GSM_CHECK_RES(cmd, expect) (expectGSMResponse(CR LF expect CR LF, sizeof(expect) + 4) != 0)
#define CMD_GSM_CHECK(cmd, expect) \
	CMD_GSM(cmd); \
	if (CMD_GSM_CHECK_RES(cmd, expect)) \
		UART_WRITELN(uartPC, "BAD RESPONSE: Expected: " expect);

void setupGSM() {
	char res[GSM_MAX_RES_LEN] = {0};

	UART_WRITELN(uartPC, "Initializing GSM Connection");
	UART_WRITELN(uartPC, "Verifying GPRS Attachment");

	CMD_GSM("AT+CGATT?");
	waitForGSMResponse(res);
	if (strcmp(res, "+CGATT: 0") != 0)
		UART_WRITELN(uartPC, "BAD RESPONSE 1");
	waitForGSMResponse(res);
	if (strcmp(res, "OK") != 0)
		UART_WRITELN(uartPC, "NO OK");
	CMD_GSM("AT+CGATT?");
	waitForGSMResponse(res);
	if (strcmp(res, "+CGATT: 1") != 0)
		UART_WRITELN(uartPC, "BAD RESPONSE 2");
	waitForGSMResponse(res);
	if (strcmp(res, "+CGATT: 0") != 0)
		UART_WRITELN(uartPC, "BAD RESPONSE 3");

	UART_WRITELN(uartPC, "TEST DONE");


	// faketest (just verify my macros work)
	CMD_GSM_CHECK("AT+CGATT?", "+CGATT: 1");
	CMD_GSM_CHECK("AT+CGATT?", "+CGATT: 0"); // should fail

	// Verify GSM setup
	CMD_GSM_CHECK("AT+CREG?", "+CREG: 0,1");  // verify network registration
	CMD_GSM_CHECK("AT+CGATT=1", "OK"); // Make sure GPRS is attached (necessary?)
	CMD_GSM_CHECK("AT+CGDCONT=1,\"IP\",\"breadnet\"", "OK");  // Set PDP context
	CMD_GSM_CHECK("AT+CGACT=1,1", "OK");  // Activate PDP context
}

#define DATALEN "3"
#define DATA "GET"
void requestGET() {
	CMD_GSM_CHECK("AT+SDATACONF=1,\"TCP\",\"www.google.com\",80", "OK"); // Configure connection
	CMD_GSM_CHECK("AT+SDATASTART=1,1", "OK");  // Start connection
	// TODO: Check the socket AT+SDATASTATUS=1
	CMD_GSM_CHECK("AT+SDATATSEND=1," DATALEN, ">");  // Issue the request
	CMD_GSM_CHECK(DATA CTRLZ, "OK");  // Enter the data
}

void gsmTask(UArg arg0, UArg arg1) {
	char input;

	setupUART();
	setupGSM();
	requestGET();

	while (1) {
		// Read from all the UARTS
		UART_read(uartPC, &input, 1);
		if (input == '{') {
			CMD_GSM_CHECK("AT+CGATT?", "+CGATT: 1");
			CMD_GSM_CHECK("AT+CGATT?", "+CGATT: 0");
		}
		if (input == '{') {
			setupGSM();
		}
		UART_read(uartGSM, &input, 1);
	}
}
