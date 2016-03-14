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
#include <lwip/api.h>
#include <lwip/opt.h>
#include <esp/uart.h>
#include <queue.h>
#include <dhcpserver.h>
#include <stdint.h>
#include <netbuf_helpers.h>
#include <string.h>

#define SERVER_IP "192.168.1.1"
#define SERVER_PORT 8080 
#define SSID "MSP432_Breadcrumb2"
#define PASSWORD "meepvista2" 
#define SOCKET_WAIT_TIME 10000
#define MAX_CONN 5
#define PACKET_SIZE 4096
#define ATT_CHAR '='
#define GSM_ATT_CHAR '#'
#define NUM_ATT_CHAR 3



//void rx_task(void *pvParameters);
void wifi_80_task(void *pvParameters);
void rx_task(void *pvParameters);
void configure_wifi();
void user_init(void);
void send_att_char();

void send_att_char() {
	uint8_t char_inc;
	for (char_inc = 0; char_inc < NUM_ATT_CHAR; char_inc++) {
        putchar(GSM_ATT_CHAR);
    }
}

void wifi_80_task(void *pvParameters) {
	//MEMP_NUM_TCP_PCB = 5;
	//espconn_tcp_set_max_con(5); //supress max connection notices.
	xQueueHandle * incoming_queue = (xQueueHandle *)pvParameters;
	//espconn_tcp_set_max_con(5);
	printf("---Handle in wifi 80: %p---\n", incoming_queue);
	if (incoming_queue == 0) {
		printf("---Failed to create queue!---");
	}
	struct netconn *nc = netconn_new (NETCONN_TCP);
	if(!nc) {
		printf("---Status monitor: Failed to allocate socket.---\r\n");
		return;
	}
	printf("---Binding...---\n");
	netconn_bind(nc, IP_ADDR_ANY, SERVER_PORT);
	printf("---Bound, starting listen...---\n");
	netconn_listen(nc);
	printf("---Listening...---\n");
  		
	while(1) {
		struct netconn *client = NULL;
    	err_t err = netconn_accept(nc, &client);
    	printf("---Ack connection...---\n");
    	if ( err != ERR_OK ) {
      		if(client) {
      			printf("---ERROR!!:: %d---\n", err);
				netconn_delete(client);
      		}
      		continue;
    	}
    	struct netbuf *netbuf;
    	send_att_char();
    	netconn_set_recvtimeout(client, 1000/portTICK_RATE_MS);
    	while((err = netconn_recv(client, &netbuf)) == ERR_OK) {
    		uint16_t len = netbuf_len(netbuf);
	        uint16_t offset;
	        for(offset = 0; offset<len; offset++) {
	        	putchar(netbuf_read_u8(netbuf, offset));
        	}
    	}
    	send_att_char();
    	//char buf[80];
    	//snprintf(buf, sizeof(buf), "Free heap %d bytes\r\n", (int)xPortGetFreeHeapSize());
    	//netconn_write(client, buf, strlen(buf), NETCONN_NOCOPY);
    	//char fuck[] = "fuck you\n"; 
    	//netconn_write(client, fuck, strlen(fuck), NETCONN_COPY);
    	char rx_char;
    	while(xQueueReceive( *incoming_queue, &rx_char, SOCKET_WAIT_TIME/portTICK_RATE_MS )) {
    		netconn_write(client, &rx_char, 1, NETCONN_COPY); 
    	}
		netconn_close(client);
    	while((err = netconn_delete(client)) != ERR_OK) {
    		printf("---Can't close connection: %d---\n", err);
    		vTaskDelay(1000/portTICK_RATE_MS);
    	}
    	//netbuf_free(netbuf);
    	netbuf_delete(netbuf);
    	//vTaskDelay(10000/portTICK_RATE_MS);
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
	sdk_wifi_set_opmode(SOFTAP_MODE);
    IP4_ADDR(&ap_ip.ip, 192, 168, 1, 1);
    IP4_ADDR(&ap_ip.gw, 192, 168, 1, 1);
    IP4_ADDR(&ap_ip.netmask, 255, 255, 255, 0);
    sdk_wifi_set_ip_info(1, &ap_ip);

    struct sdk_softap_config ap_config = {
        .ssid = SSID,
        .ssid_hidden = 0,
        .channel = 3,
        .ssid_len = strlen(SSID),
        .authmode = AUTH_OPEN,
        .password = PASSWORD,
        .max_connection = 5,
        .beacon_interval = 100,
    };
    sdk_wifi_softap_set_config(&ap_config);

    ip_addr_t first_client_ip;
    IP4_ADDR(&first_client_ip, 192, 168, 1, 2);
    dhcpserver_start(&first_client_ip, 6);
}

static xQueueHandle incoming_queue;

void user_init(void)
{
	//system_set_os_print(0);
	uart_set_baud(0, 115200);
	printf("---Configuring WiFi---\n");
	configure_wifi();
    printf("---SDK version:%s---\n", sdk_system_get_sdk_version());
    incoming_queue = xQueueCreate( 100, sizeof(char) );
    xTaskCreate(rx_task, (signed char *) "rx_task", 2048, &incoming_queue, 2, NULL);
    xTaskCreate(wifi_80_task, (signed char *) "wifi_80_task", 4096,&incoming_queue, 2, NULL);
    printf("---User init complete!---\n");
}

