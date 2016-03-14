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
 *  ======== uartecho.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

#include <stdint.h>

#define TASKSTACKSIZE     768

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

 UART_Handle uartPC;
 UART_Handle uartGSM;
 UART_Handle uartXBEE;

 void pcReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
 	if (count != UART_ERROR) {
 		// Write the data to the GSM
 		UART_write(uartGSM, buf, count);
 		// Write the data to the PC too
 		UART_write(uartPC, buf, count);
 		// Write the data to the PC too
 		UART_write(uartXBEE, buf, count);
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

 void xbeeReadUARTCallback(UART_Handle handle, void *buf, size_t count) {
 	if (count != UART_ERROR) {
 		// Write the data to the GSM
 		UART_write(uartPC, buf, count);
 	}
 }

 void xbeeWriteUARTCallback(UART_Handle handle, void *buf, size_t count) {
 	// Empty!
 }

 /*
  *  ======== echoFxn ========
  *  Task for this function is created statically. See the project's .cfg file.
  */
 Void echoFxn(UArg arg0, UArg arg1)
 {
     char input;
     UART_Params uartPCParams;
     UART_Params uartGSMParams;
     UART_Params uartXBEEParams;

     const char echoPrompt[] = "\fEchoing characters:\r\n";

     // Create a UART for PC
     UART_Params_init(&uartPCParams);
     uartPCParams.readMode = UART_MODE_CALLBACK;
     uartPCParams.writeMode = UART_MODE_CALLBACK;
     uartPCParams.readCallback = &pcReadUARTCallback;
     uartPCParams.writeCallback = &pcWriteUARTCallback;
     uartPCParams.writeDataMode = UART_DATA_BINARY;
     uartPCParams.readDataMode = UART_DATA_BINARY;
     uartPCParams.readReturnMode = UART_RETURN_NEWLINE;
     uartPCParams.readEcho = UART_ECHO_OFF;
     uartPCParams.baudRate = 115200;
     uartPC = UART_open(Board_UART0, &uartPCParams);

     if (uartPC == NULL) {
         System_abort("Error opening the UART");
     }


     // Create a UART for GSM
 	UART_Params_init(&uartGSMParams);
 	uartGSMParams.readMode = UART_MODE_CALLBACK;
     uartGSMParams.writeMode = UART_MODE_CALLBACK;
 	uartGSMParams.readCallback = &gsmReadUARTCallback;
 	uartGSMParams.writeCallback = &gsmWriteUARTCallback;
 	uartGSMParams.writeDataMode = UART_DATA_BINARY;
 	uartGSMParams.readDataMode = UART_DATA_BINARY;
 	uartGSMParams.readReturnMode = UART_RETURN_NEWLINE;
 	uartGSMParams.baudRate = 115200;
 	uartGSM = UART_open(Board_UART1, &uartGSMParams);

 	if (uartGSM == NULL) {
 		System_abort("Error opening the GSM UART");
 	}

     // Create a UART for GSM
 	UART_Params_init(&uartXBEEParams);

 	uartXBEEParams.readMode = UART_MODE_CALLBACK;
     uartXBEEParams.writeMode = UART_MODE_CALLBACK;
 	uartXBEEParams.readCallback = &xbeeReadUARTCallback;
 	uartXBEEParams.writeCallback = &xbeeWriteUARTCallback;
 	uartXBEEParams.writeDataMode = UART_DATA_BINARY;
 	uartXBEEParams.readDataMode = UART_DATA_BINARY;
 	uartXBEEParams.readReturnMode = UART_RETURN_NEWLINE;
 	uartXBEEParams.baudRate = 115200;
 	uartXBEE = UART_open(Board_UART3, &uartXBEEParams);

 	if (uartXBEE == NULL) {
 		System_abort("Error opening the GSM UART");
 	}

     UART_write(uartPC, echoPrompt, sizeof(echoPrompt));


     while (1) {

     	// Read from all the UARTS
         UART_read(uartPC, &input, 1);
         UART_read(uartGSM, &input, 1);
         UART_read(uartXBEE, &input, 1);


     }
 }

/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();

    /* Construct BIOS objects */
    Task_Params taskParams;

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    taskParams.instance->name = "echo";
    Task_construct(&task0Struct, (Task_FuncPtr)echoFxn, &taskParams, NULL);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);


    /* Start BIOS */
    BIOS_start();

    return (0);
}
