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

#include "gsm.h"


UART_easywrite(UART_Handle handle, void *buf) {
	UART_write(handle, buf, sizof(buf));
}

void pcReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
	if (count != UART_ERROR) {
		// Write the data to the GSM
		UART_write(uartGSM, buf, count);
		// Write the data to the PC too
		UART_write(uartPC, buf, count);
	}
}

void pcWriteUARTCallback(UART_Handle handle, void *buf, size_t count) {
	// Empty!
}

void gsmReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
	if (count != UART_ERROR) {
		// Write the data to the GSM
		UART_write(uartPC, buf, count);
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

int cmdGSM(char *cmd, char *expect) {
	char input;
	unsigned i = 0;

	UART_easywrite(uartGSM, cmd);
	while (1) {
		input =
        UART_read(uartGSM, &input, 1);
	}
}

void setupGSM() {
	char buf[10];
	System_printf("Initializing GSM Connection\n");
	System_printf("Verifying GPRS Attachment\n");
	cmdGSM
	UART_easywrite(uartGSM, "AT+CGATT?\r");
    UART_read(uartGSM, &input, 1);
}

void requestGET(char *hostname ) {

}

void gsmTask(UArg arg0, UArg arg1)
{
    char input;

    setupUART();
    setupGSM();

    while (1) {
    	// Read from all the UARTS
        UART_read(uartPC, &input, 1);
        UART_read(uartGSM, &input, 1);
    }
}