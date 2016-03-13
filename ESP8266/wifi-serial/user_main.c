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

#include <espressif/esp_common.h>
#include <FreeRTOS.h>
#include <task.h>
#include <lwip/sockets.h>
#include <esp/uart.h>
#include <queue.h>
#include <dhcpserver.h>
#include <stdint.h>
#include <string.h>

#define SERVER_IP "192.168.1.1"
#define SERVER_PORT 8080 
#define SSID "MSP432_Breadcrumb"
#define PASSWORD "meepvista2" 
#define BUFFER_SIZE 5000
#define SOCKET_WAIT_TIME 3000
#define MAX_CONN 5
#define PACKET_SIZE 4096
#define ATT_CHAR '='
#define NUM_ATT_CHAR 3

//void rx_task(void *pvParameters);
void wifi_80_task(void *pvParameters);
void rx_task(void *pvParameters);
void configure_wifi();
void user_init(void);


void wifi_80_task(void *pvParameters) {
	xQueueHandle * incoming_queue = (xQueueHandle *)pvParameters;
	printf("---Handle in wifi 80: %p---\n", incoming_queue);
	if (incoming_queue == 0) {
		printf("---Failed to create queue!---");
	}
	while(1) {
		/* Create socket for incoming connections 
		 * Currently this is only going to serve a tcp connecection at a time. 
		 * Put in task perhaps???
		 */
		int32_t listenfd;
		int32_t ret;
		struct sockaddr_in server_addr,remote_addr;

		/* Construct local address structure */
		memset(&server_addr, 0, sizeof(server_addr)); /* Zero out structure */
		server_addr.sin_family = AF_INET;            /* Internet address family */
		server_addr.sin_addr.s_addr = INADDR_ANY;   /* Any incoming interface */
		server_addr.sin_len = sizeof(server_addr);
		server_addr.sin_port = htons(SERVER_PORT); /* Local port */

		char recv_buf[PACKET_SIZE];

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
          		printf("ESP8266 TCP server task > bind fail\n");
          		vTaskDelay(1000/portTICK_RATE_MS);
     		}
 		} while(ret != 0);
 		do {
	    	/* Listen to the local connection */
	    	ret = listen(listenfd, MAX_CONN); //changed from MAX_CONN to 1???
    		if (ret != 0) {
        		printf("ESP8266 TCP server task > failed to set listen queue!\n");
        		vTaskDelay(1000/portTICK_RATE_MS);
    		}
		} while(ret != 0);
		int32_t client_sock;
		int32_t len = sizeof(struct sockaddr_in);
		for (;;) {
			/*block here waiting remote connect request*/
   			if ((client_sock = accept(listenfd, (struct sockaddr *)&remote_addr, (socklen_t *)&len)) < 0) {
       			//printf("ESP8266 TCP server task > accept fail\n");
				continue; 
			}

			//put this in struct perhaps
			//printf("ESP8266 TCP server task > Client from %s %d\n", inet_ntoa(remote_addr.sin_addr), htons(remote_addr.sin_port));
			//printf("%s\n", );
			//How to limit phone to 128 byet packet???
			//char *recv_buf = (char *)zalloc(PACKET_SIZE); //can this be larger...
			uint32_t recbytes;
			uint32_t readBytes = 0;
			while ((recbytes = read(client_sock , recv_buf+readBytes, PACKET_SIZE-readBytes)) > 0) {
      			recv_buf[recbytes] = 0;	
      			readBytes+= recbytes;
			}
			printf("###%s,%d,%s###\n", inet_ntoa(remote_addr.sin_addr), htons(remote_addr.sin_port), recv_buf);
  			if (incoming_queue != 0) {
				char * rx_char;
				printf("---Waiting for response...---\n");
				int bytes_written_total = 0;
  				while(xQueueReceive(*incoming_queue, &rx_char, SOCKET_WAIT_TIME / portTICK_RATE_MS)) {
  					
  					//printf("\n---\nPacket data: %s \n---\n", packet->tcp_data);
  					//ETS_UART_INTR_DISABLE();
  					int bytes_written = write(client_sock, rx_char, sizeof(char));
  					int secno = errno;
  					//ETS_UART_INTR_ENABLE();
  					printf("---hmmm: %d\n---", bytes_written);
  					if (bytes_written > 0) {
  						bytes_written_total += bytes_written; 
  					}
  					else {
  						printf("ERROR: %d\n", secno);
  					}
  					//printf("!--Bytes written: %d--!\n", bytes_written);
  					//free(packet);
  				}
  				printf("!--Bytes written: %u--!\n", bytes_written_total);
  			}
			//free(recv_buf);
  			uint32_t i;
			for(i = 0; i < PACKET_SIZE; i++) {
				recv_buf[i] = 0;
			}
			close(client_sock);
			printf("---Closed socket---");
			vTaskDelay(1000 / portTICK_RATE_MS);
		}
		printf("---UH OH----\n");
	}
}


void rx_task(void * pvParameters) {
	uint8_t attention_count = 0;
	xQueueHandle * incoming_queue = (xQueueHandle *)pvParameters;
	while(1) {
		char rec_char = getchar();
        if (rec_char == ATT_CHAR) {
            attention_count++;
            if(attention_count == NUM_ATT_CHAR*2) {
                attention_count = 0;
            }
        }
        else {
            if (attention_count >= NUM_ATT_CHAR && attention_count < NUM_ATT_CHAR*2) {
                xQueueSend(*incoming_queue, &rec_char, 0);
            }
            else{
                attention_count = 0;
            }
        }
	}
}


void configure_wifi() {
	struct ip_info ap_ip;
    IP4_ADDR(&ap_ip.ip, 192, 168, 1, 1);
    IP4_ADDR(&ap_ip.gw, 0, 0, 0, 0);
    IP4_ADDR(&ap_ip.netmask, 255, 255, 255, 0);
    sdk_wifi_set_ip_info(1, &ap_ip);

    struct sdk_softap_config ap_config = {
        .ssid = SSID,
        .ssid_hidden = 0,
        .channel = 3,
        .ssid_len = strlen(SSID),
        .authmode = AUTH_WPA_WPA2_PSK,
        .password = PASSWORD,
        .max_connection = 3,
        .beacon_interval = 100,
    };
    sdk_wifi_softap_set_config(&ap_config);

    ip_addr_t first_client_ip;
    IP4_ADDR(&first_client_ip, 172, 16, 0, 2);
    dhcpserver_start(&first_client_ip, 4);
}

static xQueueHandle incoming_queue;

void user_init(void)
{
	uart_set_baud(0, 115200);
	printf("---Configuring WiFi---\n");
	configure_wifi();
    printf("---SDK version:%s---\n", sdk_system_get_sdk_version());
    while(!sdk_wifi_set_opmode(SOFTAP_MODE)){
    	printf("Setting up wifi...");
    };
    incoming_queue = xQueueCreate( 100, sizeof(char) );
    xTaskCreate(rx_task, (signed char *) "rx_task", 512, &incoming_queue, 2, NULL);
    xTaskCreate(wifi_80_task, (signed char *) "wifi_80_task", 2048, &incoming_queue, 2, NULL);
    printf("---User init complete!---\n");
}

