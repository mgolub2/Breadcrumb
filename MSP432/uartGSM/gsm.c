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
#define ESC "\033"
#define CR "\015"
#define LF "\012"

#define CTRLZ_C '\032'
#define ESC_C '\033'
#define CR_C '\015'
#define LF_C '\012'

// constrant string macro for lazy people not wanting to count the length
#define CSTR(str) str, sizeof(str)-1

volatile int commandModeOn = 0;
void pcReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
	if (count != UART_ERROR && count > 0) {
		// Write the data to the PC too
		UART_writePolling(uartPC, buf, count);

		if (commandModeOn && ((char*) buf)[0] != '!')
			// Write the data to the GSM
			UART_writePolling(uartGSM, buf, count);
	}
}

#define GSM_MAX_RES_LEN 256
volatile char gsmRes[GSM_MAX_RES_LEN] = { 0 };
volatile unsigned gsmResLen = 0;
// True if the result was truncated in the response buffer
volatile unsigned resTrunc = 0;
enum GSM_RES_STATES {
	GSM_WAITING_FOR_START_CR,
	GSM_WAITING_FOR_START_LF,
	GSM_WAITING_FOR_END_CR,
	GSM_WAITING_FOR_END_LF,
	GSM_WAITING_FOR_END_SPACE // only during data entry mode
};
volatile enum GSM_RES_STATES gsmResponseState = GSM_WAITING_FOR_START_CR;

inline int gsmResponseReady() {
	return gsmResponseState == GSM_WAITING_FOR_START_CR && gsmResLen > 0;
}

void clearGSMResponseSafe() {
	gsmResLen = 0;
	resTrunc = 0;
}

void clearGSMResponse() {
	clearGSMResponseSafe();
	// desperate
	gsmResponseState = GSM_WAITING_FOR_START_CR;
}

#define GSM_READ_MAX_TIMEOUT_MS 30000
#define GSM_READ_DEFAULT_TIMEOUT_MS 5000
Timer_Handle gsmReadTimer;
volatile int gsmReadTimeout = 0;
void gsmReadTimeoutIsr(UArg arg) {
	if (gsmReadTimeout < GSM_READ_MAX_TIMEOUT_MS)
		gsmReadTimeout++;
}

inline int gsmReadTimedout(int timeout_ms) {
	if (timeout_ms < 0)
		return 0;

	return gsmReadTimeout >= timeout_ms;
}

inline void gsmReadTimeoutReset() {
	gsmReadTimeout = 0;
}

// res MUST have length = GSM_MAX_RES_LEN
// returns -1 on a timeout
int waitForGSMResponse(char *res, int timeout_ms) {
	char input;

	Timer_start(gsmReadTimer);
	while (!gsmResponseReady()) {
		if (gsmReadTimedout(timeout_ms)) {
			Timer_stop(gsmReadTimer);
			gsmReadTimeoutReset();
			clearGSMResponse();
			if (res != NULL)
				res[0] = '\0';

			UART_writePolling(uartPC, CSTR("GSM read timeout\r\n"));
			return -1;
		}
		UART_read(uartGSM, &input, 1);
	}
	Timer_stop(gsmReadTimer);

	if (res != NULL)
		strcpy(res, (const char *) gsmRes);

	clearGSMResponse();
	return 0;
}

void parseGSMResponse(void *buf, size_t count) {
	int processed = 0;
	while (processed < count) {
		char cur = ((char *) buf)[processed];
		if (cur == '\0') {
			processed++;
			continue;
		}
		if (gsmResponseState == GSM_WAITING_FOR_START_CR) {
			if (cur == CR_C)
				gsmResponseState = GSM_WAITING_FOR_START_LF;
		} else if (gsmResponseState == GSM_WAITING_FOR_START_LF) {
			if (cur == LF_C) {
				gsmResponseState = GSM_WAITING_FOR_END_CR;
				// reset the response
				clearGSMResponseSafe();
			} else if (cur != CR_C) {
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

				if (cur == '>' && gsmResLen == 1)
					gsmResponseState = GSM_WAITING_FOR_END_SPACE;
				gsmRes[gsmResLen] = '\0';
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
		} else if (gsmResponseState == GSM_WAITING_FOR_END_SPACE) {
			if (cur != LF_C && cur != CR_C) {
				// copy
				if (gsmResLen == GSM_MAX_RES_LEN - 1) {
					resTrunc = 1;
					break;
				}
				gsmRes[gsmResLen] = cur;
				gsmResLen++;
			}

			if (cur == ' ' && gsmResLen == 2) {
				gsmResponseState = GSM_WAITING_FOR_START_CR;
				gsmRes[gsmResLen] = '\0';
			} else {
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

UART_Params uartGSMParams;
void setupUART() {
	UART_Params uartPCParams;

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
	clearGSMResponse();
	UART_writePolling(uartPC, cmd, cmdlen);
	UART_writePolling(uartPC, "\r\n", 2);
	UART_writePolling(uartGSM, cmd, cmdlen);
	UART_writePolling(uartGSM, CR, 1);
}

#define EAT_OK 1
#define NO_EAT_OK 0
// value is guarenteed to be different from any strcmp return value in any sane implementation
#define CMD_VERIFY_TIMEDOUT (int)(1 << 9)
// lengths don't include the null terminator
// Returns the strcmp result of the received response vs the expected response
int cmdVerifyResponse(char *cmd, unsigned cmdlen, char *expect, unsigned explen,
		unsigned eatOK, int timeout_ms, int n) {
	char res[GSM_MAX_RES_LEN] = { 0 };

	if (cmd != NULL)
		cmdGSM(cmd, cmdlen);

	if (waitForGSMResponse(res, timeout_ms) == -1)
		return CMD_VERIFY_TIMEDOUT;

	if (eatOK)
		waitForGSMResponse(NULL, timeout_ms);

	if (n)
		return strncmp(res, expect, explen);
	else
		return strcmp(res, expect);
}

#define CMD_GSM_VERIFY(cmd, expect, eatOK, timeout_ms) cmdVerifyResponse(CSTR(cmd), CSTR(expect), eatOK, timeout_ms, 0)
#define CMD_GSM_VERIFY_N(cmd, expect, eatOK, timeout_ms) cmdVerifyResponse(CSTR(cmd), CSTR(expect), eatOK, timeout_ms, 1)

// cmd, expect, and error_print must be const strings
#define CMD_GSM_RET_ON_FAILURE(cmd, expect, eatOK, error_print, timeout_ms) \
	if (CMD_GSM_VERIFY(cmd, expect, eatOK, timeout_ms)) { \
		UART_writePolling(uartPC, CSTR(error_print)); \
		return -1; \
	}

// returns 1 when the gsm is ready to communicate, else 0
// it it's booting, it will wait until boot is complete and it is ready
int gsmReady() {
	char res[GSM_MAX_RES_LEN] = { 0 };

	UART_writePolling(uartPC, CSTR("Checking if GSM exists\r\n"));
	while (1) {
		// check if we can already communicate
		if (CMD_GSM_VERIFY("AT+SIND=1023", "OK", NO_EAT_OK, 3000) == 0)
			return 1;

		// check if it's booting
		if (waitForGSMResponse(res, 10000))
			// not booting, so see if it responds to a command
			return CMD_GSM_VERIFY("AT+SIND=1023", "OK", NO_EAT_OK, 5000)
					!= CMD_VERIFY_TIMEDOUT ;

		// it's booting! wait until it's ready
		if (!strncmp(res, CSTR("+SIND")))
			if (res[7] == '4')
				return CMD_GSM_VERIFY("AT+SIND=1023", "OK", NO_EAT_OK, 10000)
						!= CMD_VERIFY_TIMEDOUT ;
	}

	return 1;
}

int setupGSM() {
	UART_writePolling(uartPC, CSTR("Initializing GSM Connection\r\n"));

//	int res = CMD_VERIFY_TIMEDOUT;
//	while (res == CMD_VERIFY_TIMEDOUT) {
//		res = CMD_GSM_VERIFY("AT+SIND=1023", "OK", NO_EAT_OK, 30000);
//	}

	//CMD_GSM_RET_ON_FAILURE("AT+SIND=1023", "OK", NO_EAT_OK, "No response from GSM, must be on a non-GSM node\r\n", 30000);
	if (!gsmReady())
		return -1;

	UART_writePolling(uartPC, CSTR("Verifying GPRS attachment\r\n"));
	if (CMD_GSM_VERIFY("AT+CGATT?", "+CGATT: 1", EAT_OK,
			GSM_READ_DEFAULT_TIMEOUT_MS)) {
		// Cell not attached
		UART_writePolling(uartPC, CSTR("Not attached, attaching GPRS now\r\n"));
		CMD_GSM_RET_ON_FAILURE("AT+CGATT=1", "OK", NO_EAT_OK,
				"Failed to attach GPRS\r\n", GSM_READ_DEFAULT_TIMEOUT_MS);
	}

	UART_writePolling(uartPC, CSTR("Verifying network connection\r\n"));
	CMD_GSM_RET_ON_FAILURE("AT+CREG?", "+CREG: 0,1", EAT_OK,
			"Not registered with network\r\n", GSM_READ_DEFAULT_TIMEOUT_MS);

	// Can't set this twice (just fyi)
	UART_writePolling(uartPC, CSTR("Setting PDP context\r\n"));
	CMD_GSM_RET_ON_FAILURE("AT+CGDCONT=1,\"IP\",\"breadnet\"", "OK", NO_EAT_OK,
			"Failed to set PDP context\r\n", GSM_READ_DEFAULT_TIMEOUT_MS);
	CMD_GSM_RET_ON_FAILURE("AT+CGACT=1,1", "OK", NO_EAT_OK,
			"Failed to activate PDP context\r\n", GSM_READ_DEFAULT_TIMEOUT_MS);

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

	gsmReadTimer = Timer_create(Timer_ANY, gsmReadTimeoutIsr, &timerParams,
			&eb);

	return gsmReadTimer == NULL;
}

int gsmUARTReconfig() {
	UART_Handle uartGSMTmp;

	// max GSM baud 460800
	CMD_GSM_RET_ON_FAILURE("AT+IPR=115200", "OK", NO_EAT_OK,
			"Failed to reconfigure gsmUART\r\n", 30000);

	uartGSMParams.baudRate = 115200;
	uartGSMTmp = UART_open(Board_UART1, &uartGSMParams);

	if (uartGSMTmp == NULL) {
		return -1;
	}

	UART_close(uartGSM);
	uartGSM = uartGSMTmp;
	return 0;
}

/* reverse:  reverse string s in place */
void reverse(char s[]) {
	int i, j;
	char c;

	for (i = 0, j = strlen(s) - 1; i < j; i++, j--) {
		c = s[i];
		s[i] = s[j];
		s[j] = c;
	}
}

/* K&R itoa:  convert n to characters in s */
// what about overflow?
void itoa(int n, char s[]) {
	int i, sign;

	if ((sign = n) < 0) /* record sign */
		n = -n; /* make n positive */
	i = 0;
	do { /* generate digits in reverse order */
		s[i++] = n % 10 + '0'; /* get next digit */
	} while ((n /= 10) > 0); /* delete it */
	if (sign < 0)
		s[i++] = '-';
	s[i] = '\0';
	reverse(s);
}

int requestGET() {
	char *data = "GET / HTTP/1.1\r\nHost: google.com\r\n\r\n";
	//char *data = "G";

	UART_writePolling(uartPC, CSTR("Connecting to Google\r\n"));

	if (CMD_GSM_VERIFY_N("AT+SDATASTATUS=1", "+SOCKSTATUS:  1,1", EAT_OK,
			3000)) {
		CMD_GSM_RET_ON_FAILURE("AT+SDATACONF=1,\"TCP\",\"www.google.com\",80",
				"OK", NO_EAT_OK, "TCP configuration failed\r\n",
				GSM_READ_DEFAULT_TIMEOUT_MS);
		CMD_GSM_RET_ON_FAILURE("AT+SDATASTART=1,1", "OK", NO_EAT_OK,
				"TCP connect failed\r\n", GSM_READ_DEFAULT_TIMEOUT_MS);
	}

	// get th length of the data, as both an int and string
	int datalen = strlen(data);
	char datalen_str[32] = { 0 };
	itoa(datalen, datalen_str);

	char sendstring[128] = { 0 };
	strcpy(sendstring, "AT+SDATATSEND=1,");
	strcpy(sendstring + strlen(sendstring), datalen_str);

	//snprintf(sendstring, 128, "AT+SDATATSEND=1,%i", datalen);


	if (cmdVerifyResponse(sendstring, strlen(sendstring), CSTR("> "), NO_EAT_OK,
	GSM_READ_DEFAULT_TIMEOUT_MS, 0)) {
		UART_writePolling(uartPC, CSTR("Request to GSM failed\r\n"));
		return -1;
	}
	//CMD_GSM_RET_ON_FAILURE("AT+SDATATSEND=1," DATALEN, "> ", NO_EAT_OK, "Request to GSM failed\r\n", GSM_READ_DEFAULT_TIMEOUT_MS);

	// This is pretty dangerous and stupid, but so much faster than a full copy and easier than two writes
	//data[datalen] = CTRLZ_C;

	clearGSMResponse();
	char ctlz = (char)26;
	UART_writePolling(uartPC, data, datalen);
	UART_writePolling(uartGSM, data, datalen);
	int i = 0;
	while (i < 1000000)
		i++;
	UART_writePolling(uartPC, &ctlz, 1);
	UART_writePolling(uartGSM, &ctlz, 1);
	waitForGSMResponse(NULL, GSM_READ_DEFAULT_TIMEOUT_MS); // OK

//	if (cmdVerifyResponse(data, datalen+1, CSTR("OK"), NO_EAT_OK, GSM_READ_DEFAULT_TIMEOUT_MS, 0)) {
//		UART_writePolling(uartPC, CSTR("Entering data failed\r\n"));
//		UART_writePolling(uartGSM, CTRLZ, 1);
////		CMD_GSM_RET_ON_FAILURE(CTRLZ, "OK", NO_EAT_OK, "End CTRLZ didn't help\r\n",
////				GSM_READ_DEFAULT_TIMEOUT_MS);
//	}

	//data[datalen] = '\0';
	//CMD_GSM_RET_ON_FAILURE(DATA CTRLZ, "OK", NO_EAT_OK, "Entering data failed\r\n", GSM_READ_DEFAULT_TIMEOUT_MS);

	CMD_GSM_RET_ON_FAILURE(NULL, "+STCPD:1", NO_EAT_OK, "No data response\r\n",
			10000);

	while (!cmdVerifyResponse(CSTR("AT+SDATATREAD=1"), CSTR("+CME ERROR: 4"), NO_EAT_OK, GSM_READ_DEFAULT_TIMEOUT_MS, 0)) {

	}

	return 0;
}

void gsmTask(UArg arg0, UArg arg1) {
	char input;

	int timerFail = setupTimer();
	setupUART();
	if (timerFail)
		UART_writePolling(uartPC, CSTR("Failed to create timer\r\n"));
//	if (setupGSM())
//		UART_writePolling(uartPC, CSTR("Failed to setup GSM\r\n"));
//	if (gsmUARTReconfig())
//		UART_writePolling(uartPC, CSTR("Failed reconfigure GSM UART\r\n"));
//	if (requestGET())
//		UART_writePolling(uartPC, CSTR("GET attempt failed\r\n"));

	while (1) {
		// Read from all the UARTS
		UART_read(uartPC, &input, 1);
		if (input == '!') {
			UART_writePolling(uartPC, CSTR("\r\n"));
			// toggle command mode
			if (commandModeOn) {
				UART_writePolling(uartPC, CSTR("Turning command mode off\r\n"));
				commandModeOn = 0;
			} else {
				UART_writePolling(uartPC, CSTR("Turning command mode on\r\n"));
				commandModeOn = 1;
			}
			input = '\0';
		} else if (!commandModeOn) {
			if (input == 'a') {
				UART_writePolling(uartPC, CSTR("\r\n"));
				cmdGSM(CSTR("AT"));
				input = '\0';
			} else if (input == 't') {
				UART_writePolling(uartPC, CSTR("\r\n"));
				cmdGSM(CSTR("AT+SIND?"));
				input = '\0';
			} else if (input == 's') {
				UART_writePolling(uartPC, CSTR("\r\n"));
				if (setupGSM())
					UART_writePolling(uartPC, CSTR("Failed to setup GSM\r\n"));
				input = '\0';
			} else if (input == 'g') {
				UART_writePolling(uartPC, CSTR("\r\n"));
				if (requestGET())
					UART_writePolling(uartPC, CSTR("GET attempt failed\r\n"));
				input = '\0';
			} else if (input == 'u') {
				UART_writePolling(uartPC, CSTR("\r\n"));
				if (gsmUARTReconfig())
					UART_writePolling(uartPC,
							CSTR("Failed reconfigure GSM UART\r\n"));
				input = '\0';
			} else if (input == 'b') {
				UART_writePolling(uartPC, CSTR("\r\n"));
				cmdGSM(CSTR("AT+IPR?"));
				input = '\0';
			} else if (input == ESC_C) {
				UART_writePolling(uartPC, CSTR("\r\n"));
				UART_writePolling(uartGSM, CSTR(ESC));
				input = '\0';
			} else if (input == CTRLZ_C) {
				UART_writePolling(uartPC, CSTR("\r\n"));
				UART_writePolling(uartGSM, CSTR(CTRLZ));
				input = '\0';
			} else if (input == 'n') {
				CMD_GSM_VERIFY_N("AT+SDATASTATUS=1", "+SOCKSTATUS:  1,1", EAT_OK,
									3000);
				input = '\0';
			}
		}
		UART_read(uartGSM, &input, 1);
	}
}
