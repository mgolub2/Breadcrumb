/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "esp_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "uart.h"
#include "udns.h"

#define SERVER_IP "192.168.1.1"
#define SERVER_PORT 80 
#define SSID "MSP432_Breadcrumb"
#define PASSWORD "meepvista2" 

void ICACHE_FLASH_ATTR rx_task(void *pvParameters);
void ICACHE_FLASH_ATTR tx_task(void *pvParameters);
void ICACHE_FLASH_ATTR configure_wifi();
void ICACHE_FLASH_ATTR config_custom_uart0();
void user_init(void);

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/

void uart0_rx_intr_handler (void *para) {
	//do something here
}

void ICACHE_FLASH_ATTR rx_task(void *pvParameters) {
	/*printf("I'm a rx\n");
	while(1) {
    	int j = 0;
    	int i;
    	for (i = 0; i < 100000000; i++) {
    		j ++;
    	}
    	printf("J is L %d\n", j);
    };*/
}

void ICACHE_FLASH_ATTR wifi_80_task(void *pvParameters) {
	printf("I'm a tx\n");
	int32 listenfd;
	int32 ret;
	struct sockaddr_in server_addr,remote_addr;
	int stack_counter=0;
	/* Construct local address structure */
	memset(&server_addr, 0, sizeof(server_addr)); /* Zero out structure */
	server_addr.sin_family = AF_INET;            /* Internet address family */
	server_addr.sin_addr.s_addr = INADDR_ANY;   /* Any incoming interface */
	server_addr.sin_len = sizeof(server_addr);
	server_addr.sin_port = htons(SERVER_PORT); /* Local port */
	while(1) {
		/* Create socket for incoming connections 
		 * Currently this is only going to serve a tcp connecection at a time. 
		 * Put in task perhaps???
		 */
		do {
			listenfd = socket(AF_INET, SOCK_STREAM, 0);
			if (listenfd == -1) {
				//printf("ESP8266 TCP server task > socket error\n”);
			 	vTaskDelay(1000/portTICK_RATE_MS);
			}
		} while(listenfd == -1);
		/* Bind to the local port */
 		do {
     		ret = bind(listenfd, (struct sockaddr *)&server_addr,
			sizeof(server_addr));
     		if (ret != 0) {
          		//printf("ESP8266 TCP server task > bind fail\n”);
          		vTaskDelay(1000/portTICK_RATE_MS);
     		}
 		} while(ret != 0);
 		do {
	    	/* Listen to the local connection */
	    	ret = listen(listenfd, 1); //changed from MAX_CONN to 1???
    		if (ret != 0) {
        		//printf("ESP8266 TCP server task > failed to set listen queue!\n");
        		vTaskDelay(1000/portTICK_RATE_MS);
    		}
		} while(ret != 0);
		int32 client_sock;
		int32 len = sizeof(struct sockaddr_in);
		for (;;) {
			/*block here waiting remote connect request*/
   			if ((client_sock = accept(listenfd, (struct sockaddr *)&remote_addr, (socklen_t *)&len)) < 0) {
       			//printf("ESP8266 TCP server task > accept fail\n");
				continue; 
			}
			//put this in struct perhaps
			//printf("ESP8266 TCP server task > Client from %s %d\n", inet_ntoa(remote_addr.sin_addr), htons(remote_addr.sin_port));
			//How to limit phone to 128 byet packet???
			char *recv_buf = (char *)zalloc(1024); //can this be larger...
			uint32_t recbytes;
			while ((recbytes = read(client_sock , recv_buf, 1024)) > 0) {
      			recv_buf[recbytes] = 0;
      			printf("###%s###", recv_buf);
      			//printf("ESP8266 TCP server task > read data success %d!\nESP8266 TCP server task > %s\n", recbytes, recv_buf);
			}
			//printf("Free'd buffer!");
			free(recv_buf);
			if (recbytes <= 0) {
				//printf("ESP8266 TCP server task > read data fail!\n");
				close(client_sock);
			}
		}
	}
}

void ICACHE_FLASH_ATTR config_custom_uart0() {
	//Configure uart 0 how we want. 
	UART_ConfigTypeDef uart_config;
	uart_config.baud_rate = 115200;
	uart_config.data_bits = UART_WordLength_8b;
	uart_config.parity = USART_Parity_None;
	uart_config.stop_bits = USART_StopBits_1;
	uart_config.flow_ctrl = USART_HardwareFlowControl_None;
	uart_config.UART_RxFlowThresh = 120;
	uart_config.UART_InverseMask  = UART_None_Inverse;
	UART_ParamConfig(UART0, &uart_config);
	UART_IntrConfTypeDef uart_intr;
	uart_intr.UART_IntrEnMask = UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA
	| UART_RXFIFO_FULL_INT_ENA | UART_TXFIFO_EMPTY_INT_ENA;
	uart_intr.UART_RX_FifoFullIntrThresh = 10;
	uart_intr.UART_RX_TimeOutIntrThresh = 2;
	uart_intr.UART_TX_FifoEmptyIntrThresh = 20;
	UART_IntrConfig(UART0, &uart_intr);
	UART_SetPrintPort(UART0);
	UART_intr_handler_register(uart0_rx_intr_handler, NULL);
	ETS_UART_INTR_ENABLE();
}

void configure_wifi() {
	struct softap_config *config = (struct softap_config *)zalloc(sizeof(struct
 		softap_config));
 	wifi_softap_get_config(config); // Get soft-AP config first.
  	sprintf(config->ssid, SSID);
  	sprintf(config->password, PASSWORD);
  	config->authmode = AUTH_WPA_WPA2_PSK;
	config->ssid_len = sizeof(SSID);        // can also be 0
	config->max_connection = 4;
	wifi_softap_set_config(config); // Set ESP8266 soft-AP config
	free(config);
	wifi_softap_dhcps_stop();  // disable soft-AP DHCP server
	struct ip_info info;
	IP4_ADDR(&info.ip, 192, 168, 1, 1); // set IP
	IP4_ADDR(&info.gw, 192, 168, 1, 1); // set IP
	IP4_ADDR(&info.netmask, 255, 255, 255, 0); // set netmask
	wifi_set_ip_info(SOFTAP_IF, &info);
	struct dhcps_lease dhcp_lease;
	IP4_ADDR(&dhcp_lease.start_ip, 192, 168, 1, 2);
	IP4_ADDR(&dhcp_lease.end_ip, 192, 168, 1, 7);
	wifi_softap_set_dhcps_lease(&dhcp_lease);
	wifi_softap_dhcps_start(); // enable soft-AP DHCP server
}

void user_init(void)
{
	//config_custom_uart0();
	printf("Configuring WiFi\n");
	//dns_server_task("4");
	configure_wifi();
    printf("SDK version:%s\n", system_get_sdk_version());
    //seperate rx and tx tasks? UART INT based for TX?
    while(!wifi_set_opmode(SOFTAP_MODE)){
    	printf("Setting up wifi...");
    };
    //xTaskCreate(rx_task, "rx_task", 512, NULL, 2, NULL);
    xTaskCreate(wifi_80_task, "wifi_80_task", 2048, NULL, 2, NULL);
    xTaskCreate(dns_server_task, "dns_server_task", 512, NULL, 2, NULL);
    printf("DONE DONE\n");
}

