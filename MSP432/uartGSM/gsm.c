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

//volatile int commandModeOn = 0;

#define PC_MAX_CMD_LEN 256
volatile char pcCmd[PC_MAX_CMD_LEN] = { 0 };
volatile unsigned pcCmdLen = 0;
// True if the result was truncated in the response buffer
volatile unsigned pcCmdTrunc = 0;
enum PC_CMD_STATES {
	PC_WAITING_FOR_START,
	PC_WAITING_FOR_END
};
volatile enum PC_CMD_STATES pcCmdState = PC_WAITING_FOR_START;

int pcCmdReady() {
	return pcCmdState == PC_WAITING_FOR_START && pcCmdLen > 0;
}

void clearPCCmdSafe() {
	pcCmdLen = 0;
	pcCmdTrunc = 0;
}

void clearPCCmd() {
	clearPCCmdSafe();
	// desperate
	pcCmdState = PC_WAITING_FOR_START;
}

void pcReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
	int processed = 0;
	while (processed < count) {
		char cur = ((char *) buf)[processed];
		if (cur == '\0') {
			processed++;
			continue;
		}

		if (pcCmdState == PC_WAITING_FOR_START) {
			if (cur == '!' || cur == '@') {
				clearPCCmdSafe();
				pcCmdState = PC_WAITING_FOR_END;
				UART_writePolling(uartPC, &cur, 1);

				// copy
				if (pcCmdLen == PC_MAX_CMD_LEN - 1) {
					pcCmdTrunc = 1;
					break;
				}
				pcCmd[pcCmdLen] = cur;
				pcCmdLen++;
			}
		} else if (pcCmdState == PC_WAITING_FOR_END) {
			if (cur == '\r' || cur == '\n') {
				pcCmdState = PC_WAITING_FOR_START;
				pcCmd[pcCmdLen] = '\0';
				UART_writePolling(uartPC, "\r\n", 2);
			} else {
				UART_writePolling(uartPC, &cur, 1);

				// copy
				if (pcCmdLen == PC_MAX_CMD_LEN - 1) {
					pcCmdTrunc = 1;
					break;
				}
				pcCmd[pcCmdLen] = cur;
				pcCmdLen++;
			}
		}

		processed++;
	}
}

/********* TIMER TIMEOUTS START ***********/
#define GSM_READ_MAX_TIMEOUT_MS 30000
#define GSM_READ_DEFAULT_TIMEOUT_MS 5000
Timer_Handle gsmReadTimer;
volatile int gsmReadTimeout = 0;
void gsmReadTimeoutIsr(UArg arg) {
	if (gsmReadTimeout < GSM_READ_MAX_TIMEOUT_MS)
		gsmReadTimeout++;
	else
		Timer_stop(gsmReadTimer);
}

inline int gsmReadTimedout(int timeout_ms) {
	if (timeout_ms < 0)
		return 0;

	return gsmReadTimeout >= timeout_ms;
}

inline void gsmReadTimeoutReset() {
	gsmReadTimeout = 0;
}

#define GSM_RES_MAX_TIMEOUT 1
Timer_Handle gsmResTimer;
volatile int gsmResTimeout = 0;
void gsmResTimeoutIsr(UArg arg) {
	if (gsmResTimeout < GSM_RES_MAX_TIMEOUT)
		gsmResTimeout++;
	else
		Timer_stop(gsmResTimer);
}

inline int gsmResTimedout() {
	return gsmResTimeout >= GSM_RES_MAX_TIMEOUT ;
}

inline void gsmResTimeoutReset() {
	gsmResTimeout = 0;
}

/********* TIMER TIMEOUTS END ***********/

/********* GSM RESPONSE PARSING START ***********/
// holds the last four characters, in an int for shifting purposes
int32_t gsmLastFour = ('\r' << 8) | '\n';
int32_t gsmCmdSeparator = ('\r' << 24) | ('\n' << 16) | ('\r' << 8) | '\n'; 
int32_t gsmCmdSeparatorData = ('\r' << 24) | ('\n' << 16) | ('>' << 8) | ' '; 
int32_t gsmCmdSeparatorTimeout = ('\r' << 8) | '\n';
int32_t gsmCmdSeparatorTimeoutFlag = 0xFFFF;

#define GSM_MAX_RES_LEN 256
volatile char gsmRes[GSM_MAX_RES_LEN] = { 0 };
volatile unsigned gsmResLen = 0;
volatile unsigned gsmResTrunc = 0;

int gsmResponseReady() {
	return (gsmLastFour == gsmCmdSeparatorData || (gsmResTimedout() && ((gsmLastFour & gsmCmdSeparatorTimeoutFlag) == gsmCmdSeparatorTimeout))) && gsmResLen > 0;
}

void clearGSMResponse() {
	gsmResLen = 0;
	gsmResTrunc = 0;
}

#define GSM_RES_RING_LEN 4
volatile int resRingStart = 0;
volatile int resRingStop = 0;
char resRing[GSM_RES_RING_LEN][GSM_MAX_RES_LEN];
unsigned resLenRing[GSM_RES_RING_LEN];

void parseGSMResponse(void *buf, size_t count) {
	Timer_stop(gsmResTimer);
	gsmResTimeoutReset();

	char *gsmRes = resRingStop

	int processed = 0;
	while (processed < count) {
		char cur = ((char *) buf)[processed];
		
		if (gsmLastFour == gsmCmdSeparator || gsmLastFour == gsmCmdSeparatorData)
			clearGSMResponse();

		gsmLastFour <<= 8;
		gsmLastFour |= (int32_t)cur;

		// copy
		if (gsmResLen == GSM_MAX_RES_LEN - 1)
			gsmResTrunc = 1;

		gsmRes[gsmResLen] = cur;
		gsmResLen++;

		processed++;
	}

	gsmRes[gsmResLen] = '\0';
	Timer_start(gsmResTimer);
}
/********* GSM RESPONSE PARSING END ***********/

// res MUST have length = GSM_MAX_RES_LEN
// returns -1 on a timeout
int waitForGSMResponse(char *res, int timeout_ms) {
	char input[32];

	gsmReadTimeoutReset();
	Timer_start(gsmReadTimer);
	while (!gsmResponseReady()) {
		if (gsmReadTimedout(timeout_ms)) {
			Timer_stop(gsmReadTimer);
			gsmReadTimeoutReset();
			//clearGSMResponse();
			if (res != NULL)
				res[0] = '\0';

			UART_writePolling(uartPC, CSTR("GSM read timeout\r\n"));
			return -1;
		}

		int len;
		if (UART_control(uartGSM, UART_CMD_GETRXCOUNT, (void *)&len) == UART_STATUS_SUCCESS)
			UART_read(uartGSM, &input, len);
		else
			UART_read(uartGSM, &input, 1);
	}
	Timer_stop(gsmReadTimer);

	if (res != NULL) {
		// exclude the \r\n when possible (every time except data entry)
		if (gsmResLen > 2) {
			// we don't just have \r\n, but rather \r\n\0
			strncpy(res, (const char *) gsmRes, gsmResLen - 2);
			res[gsmResLen-1] = '\0';
		} else {
			strncpy(res, (const char *) gsmRes, gsmResLen);
			res[gsmResLen] = '\0';
		}
	}

	//clearGSMResponse();
	return 0;
}


void gsmReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
	if (count != UART_ERROR && count > 0) {
		// Write the data to the PC
		UART_writePolling(uartPC, buf, count);
		parseGSMResponse(buf, count);
	}
}

UART_Params uartGSMParams;
int setupUART() {
	UART_Params uartPCParams;

	// Create a UART for PC
	if (uartPC == NULL) {
		UART_Params_init(&uartPCParams);
		uartPCParams.readMode = UART_MODE_CALLBACK;
		uartPCParams.readCallback = &pcReadUARTCallback;
		uartPCParams.writeDataMode = UART_DATA_BINARY;
		uartPCParams.readDataMode = UART_DATA_BINARY;
		uartPCParams.baudRate = 115200;
		uartPC = UART_open(Board_UART0, &uartPCParams);

		if (uartPC == NULL)
			return -1;
	}

	// Create a UART for GSM
	if (uartGSM == NULL) {
		UART_Params_init(&uartGSMParams);
		uartGSMParams.readMode = UART_MODE_CALLBACK;
		uartGSMParams.readCallback = &gsmReadUARTCallback;
		uartGSMParams.writeDataMode = UART_DATA_BINARY;
		uartGSMParams.readDataMode = UART_DATA_BINARY;
		uartGSMParams.baudRate = 115200;
		uartGSM = UART_open(Board_UART1, &uartGSMParams);

		if (uartGSM == NULL)
			return -1;
	}

	return 0;
}

// send a command to the GSM module
void cmdGSM(char *cmd, unsigned cmdlen) {
	clearGSMResponse();
	UART_writePolling(uartPC, cmd, cmdlen);
	UART_writePolling(uartPC, "\r\n", 2);
	UART_writePolling(uartGSM, cmd, cmdlen);
	UART_writePolling(uartGSM, CR, 1);
}

// send a data to the GSM
void dataGSM(char *data, unsigned datalen) {
	clearGSMResponse();
	UART_writePolling(uartPC, data, datalen);
	UART_writePolling(uartPC, "\r\n", 2);
	UART_writePolling(uartGSM, data, datalen);
	UART_writePolling(uartGSM, CTRLZ, 1);
}

#define CMD_F_NONE 0
#define CMD_F_EAT_OK 1
#define CMD_F_CMP_N 2
#define CMD_F_DATA 4
// value is guarenteed to be different from any strcmp return value in any sane implementation
#define CMD_VERIFY_TIMEDOUT (int)(1 << 9)
// lengths don't include the null terminator
// Returns the strcmp result of the received response vs the expected response
int cmdVerifyResponse(char *cmd, unsigned cmdlen, char *expect, unsigned explen,
		int timeout_ms, unsigned flags) {
	char res[GSM_MAX_RES_LEN] = { 0 };

	if (cmd != NULL) {
		if (flags & CMD_F_DATA)
			dataGSM(cmd, cmdlen);
		else
			cmdGSM(cmd, cmdlen);
	}

	if (waitForGSMResponse(res, timeout_ms) == -1)
		return CMD_VERIFY_TIMEDOUT;

	if (flags & CMD_F_EAT_OK)
		waitForGSMResponse(NULL, timeout_ms);

	if (expect != NULL) {
		if (flags & CMD_F_CMP_N)
			return strncmp(res, expect, explen);
		else
			return strcmp(res, expect);
	}

	return 0;
}

#define CMD_GSM_VERIFY(cmd, expect, timeout_ms, flags) cmdVerifyResponse(CSTR(cmd), CSTR(expect), timeout_ms, flags)

// cmd, expect, and error_print must be const strings
#define CMD_GSM_RET_ON_FAILURE(cmd, expect, timeout_ms, flags, error_print) \
	if (CMD_GSM_VERIFY(cmd, expect, timeout_ms, flags)) { \
		UART_writePolling(uartPC, CSTR(error_print)); \
		return -1; \
	}

// returns 1 when the gsm is ready to communicate, else 0
// if it's booting, it will wait until boot is complete and it is ready
int gsmReady() {
	char res[GSM_MAX_RES_LEN] = { 0 };

	UART_writePolling(uartPC, CSTR("Checking if GSM exists\r\n"));

	UART_writePolling(uartPC, CSTR("Checking if GSM is already up\r\n"));
	if (CMD_GSM_VERIFY("AT+SIND=1023", "OK", 3000, CMD_F_NONE) == 0)
		return 1;

	UART_writePolling(uartPC, CSTR("GSM not up. Checking if it's booting\r\n"));
	if (waitForGSMResponse(res, 10000))
		// last communication attempt, could have came online after first check but before
		// this one (very rare case)
		return CMD_GSM_VERIFY("AT+SIND=1023", "OK", 1000, CMD_F_NONE)
			!= CMD_VERIFY_TIMEDOUT ;

	// TODO: have a timeout here and if times out, try to communicate once more
	UART_writePolling(uartPC, CSTR("GSM currently booting, waiting for +SIND: 4\r\n"));
	do {
		if (!strncmp(res, CSTR("+SIND")) && res[7] == '4')
			return 1;
	} while (!waitForGSMResponse(res, 5000));

	// +SIND: 4 on boot timed out, but the GSM *should* exist on this node, so do one last check
	return CMD_GSM_VERIFY("AT+SIND=1023", "OK", 3000, CMD_F_NONE) != CMD_VERIFY_TIMEDOUT;
}

#define HOST "breadnet"
int setupGSM() {
	char res[GSM_MAX_RES_LEN] = {0};

	UART_writePolling(uartPC, CSTR("Initializing GSM Connection\r\n"));

	if (!gsmReady())
		return -1;

	UART_writePolling(uartPC, CSTR("Verifying GPRS attachment\r\n"));
	if (CMD_GSM_VERIFY("AT+CGATT?", "+CGATT: 1",
				GSM_READ_DEFAULT_TIMEOUT_MS, CMD_F_EAT_OK)) {
		// Cell not attached
		UART_writePolling(uartPC, CSTR("Not attached, attaching GPRS now\r\n"));
		CMD_GSM_RET_ON_FAILURE("AT+CGATT=1", "OK", GSM_READ_DEFAULT_TIMEOUT_MS, CMD_F_NONE,
				"Failed to attach GPRS\r\n");
	}

	UART_writePolling(uartPC, CSTR("Verifying network connection\r\n"));
	CMD_GSM_RET_ON_FAILURE("AT+CREG?", "+CREG: 0,1", GSM_READ_DEFAULT_TIMEOUT_MS, CMD_F_EAT_OK,
			"Not registered with network\r\n");

	// Can't set this twice (just fyi)
	UART_writePolling(uartPC, CSTR("Checking PDP context\r\n"));
	cmdGSM(CSTR("AT+CGDCONT?"));
	if (waitForGSMResponse(res, 1000))
		return -1;

	unsigned n = 0;
	while (n < GSM_MAX_RES_LEN && !strncmp(res+n, CSTR("+CGDCONT:"))) {
		if (res[n+sizeof("+CGDCONT:")-1] != '1') {
			while (res[n] != '\n') {
				if (res[n] == '\0')
					return -1;

				n++;
			}

			n++;
		} else {
			if (strncmp(res+n+sizeof("+CGDCONT:"), CSTR(",IP," HOST))) {
				UART_writePolling(uartPC, CSTR("Setting PDP context\r\n"));
				CMD_GSM_RET_ON_FAILURE("AT+CGDCONT=1,\"IP\",\"" HOST "\"", "OK", GSM_READ_DEFAULT_TIMEOUT_MS, CMD_F_NONE,
						"Failed to set PDP context\r\n");
				CMD_GSM_RET_ON_FAILURE("AT+CGACT=1,1", "OK", GSM_READ_DEFAULT_TIMEOUT_MS, CMD_F_NONE,
						"Failed to activate PDP context\r\n");
			} else {
				UART_writePolling(uartPC, CSTR("PDP context already set\r\n"));
			}
			
			UART_writePolling(uartPC, CSTR("GSM Setup Finished\r\n"));
			return 0;
		}
	}

	return -1;
}

int setupTimers() {
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

	// 2.22ms timeout (amount of time to fill up the 32 byte ring buffer
	// on the msp432 UART at 115200 baud (excluding start/stop bits, so
	// there's some error). However if we're hitting this timeout then
	// the ring buffer is about to cycle and realistically it's too
	// late to do a read and not lose data
	timerParams.startMode = Timer_StartMode_USER;
	timerParams.period = 2222;
	timerParams.periodType = Timer_PeriodType_MICROSECS;
	timerParams.arg = 1;

	gsmResTimer = Timer_create(Timer_ANY, gsmResTimeoutIsr, &timerParams,
			&eb);

	return gsmReadTimer == NULL || gsmResTimer == NULL;
}

int gsmUARTReconfig() {
	UART_Handle uartGSMTmp;

	// max GSM baud 460800
	CMD_GSM_RET_ON_FAILURE("AT+IPR=115200", "OK", 3000, CMD_F_NONE,
			"Failed to reconfigure gsmUART\r\n");

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
	char *data = "GET / HTTP/1.1\r\nHost: www.google.com\r\n\r\n";

	UART_writePolling(uartPC, CSTR("Connecting to Google\r\n"));

	if (CMD_GSM_VERIFY("AT+SDATASTATUS=1", "+SOCKSTATUS:  1,1", 3000, CMD_F_EAT_OK | CMD_F_CMP_N)) {
		CMD_GSM_RET_ON_FAILURE("AT+SDATACONF=1,\"TCP\",\"www.google.com\",80",
				"OK", GSM_READ_DEFAULT_TIMEOUT_MS, CMD_F_NONE, "TCP configuration failed\r\n");
		CMD_GSM_RET_ON_FAILURE("AT+SDATASTART=1,1", "OK", GSM_READ_DEFAULT_TIMEOUT_MS, CMD_F_NONE,
				"TCP connect failed\r\n");
		// wait until we're actually connected
		while (CMD_GSM_VERIFY("AT+SDATASTATUS=1", "+SOCKSTATUS:  1,1,0102", 3000, CMD_F_EAT_OK | CMD_F_CMP_N));
	}

	// get the length of the data, as both an int and string
	int datalen = strlen(data);
	char datalen_str[32] = { 0 };
	itoa(datalen, datalen_str);

	char sendstring[128] = { 0 };
	strcpy(sendstring, "AT+SDATATSEND=1,");
	strcpy(sendstring + strlen(sendstring), datalen_str);
	// done getting length

	if (cmdVerifyResponse(sendstring, strlen(sendstring), CSTR("> "), GSM_READ_DEFAULT_TIMEOUT_MS, CMD_F_NONE)) {
		UART_writePolling(uartPC, CSTR("Request to GSM failed\r\n"));
		return -1;
	}

	if (cmdVerifyResponse(data, datalen, CSTR("OK"), GSM_READ_DEFAULT_TIMEOUT_MS, CMD_F_DATA)) {
		UART_writePolling(uartPC, CSTR("Entering data failed\r\n"));
		return -1;
	}

	CMD_GSM_RET_ON_FAILURE(NULL, "+STCPD:1", 10000, CMD_F_NONE, "No data response\r\n");
	
	cmdGSM(CSTR("AT+SDATATREAD=1"));

	return 0;
}

void executePCCmd(char *cmd, int cmdlen) {
	if (!strncmp(cmd, "!", 1)) {
		cmd++;
		if (!strcmp(cmd, "a")) {
			cmdGSM(CSTR("AT"));
		} else if (!strcmp(cmd, "t")) {
			cmdGSM(CSTR("AT+SIND?"));
		} else if (!strcmp(cmd, "s")) {
			if (setupGSM())
				UART_writePolling(uartPC, CSTR("Failed to setup GSM\r\n"));
		} else if (!strcmp(cmd, "g")) {
			if (requestGET())
				UART_writePolling(uartPC, CSTR("GET attempt failed\r\n"));
		} else if (!strcmp(cmd, "u")) {
			if (gsmUARTReconfig())
				UART_writePolling(uartPC,
						CSTR("Failed reconfigure GSM UART\r\n"));
		} else if (!strcmp(cmd, "b")) {
			cmdGSM(CSTR("AT+IPR?"));
		} else if (!strcmp(cmd, ESC)) {
			UART_writePolling(uartGSM, CSTR(ESC));
		} else if (!strcmp(cmd, CTRLZ)) {
			UART_writePolling(uartGSM, CSTR(CTRLZ));
		} else if (!strcmp(cmd, "n")) {
			CMD_GSM_VERIFY("AT+SDATASTATUS=1", "+SOCKSTATUS:  1,1", 3000,
					CMD_F_NONE | CMD_F_CMP_N);
		} else {
			UART_writePolling(uartPC, CSTR("Unknown PC command\r\n"));
		}
	} else if (!strncmp(cmd, "@", 1)) {
		cmd++;
		UART_writePolling(uartGSM, cmd, cmdlen-1);
		UART_writePolling(uartGSM, "\r\n", 2);
	} else {
		UART_writePolling(uartPC, CSTR("Unknown command snuck through\r\n"));
	}
}

void gsmTask(UArg arg0, UArg arg1) {
	char input[32];
	int len;

	int timerFail = setupTimers();
	while(setupUART()) ;

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
		if (UART_control(uartPC, UART_CMD_GETRXCOUNT, (void *)&len) == UART_STATUS_SUCCESS)
			UART_read(uartPC, &input, len);
		else
			UART_read(uartPC, &input, 1);

		if (UART_control(uartGSM, UART_CMD_GETRXCOUNT, (void *)&len) == UART_STATUS_SUCCESS)
			UART_read(uartGSM, &input, len);
		else
			UART_read(uartGSM, &input, 1);

		if (pcCmdReady()) {
			executePCCmd((char *)pcCmd, pcCmdLen);
			clearPCCmd();
		}
	}
}
