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
#define PASSWORD  "meepvista2" 
#define SOCKET_WAIT_TIME 7000
#define MAX_CONN 3
#define PACKET_SIZE 1024
#define ATT_CHAR '\b'
#define GSM_ATT_CHAR '\a'
#define NUM_ATT_CHAR 3

typedef struct {
	char data[PACKET_SIZE];
	uint32_t written;
} packet;


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
	printf("Struct size: %d\n", sizeof(packet));
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
    	//char rec_char;
    	packet * msg;
    	printf("---%lu left in queue---\n", uxQueueMessagesWaiting(*incoming_queue));
    	while(xQueueReceive( *incoming_queue, &msg, SOCKET_WAIT_TIME/portTICK_RATE_MS )) {
    		printf("---Packet size: %d---\n", msg->written);
    		netconn_write(client, msg->data, msg->written, NETCONN_COPY);
    		//netconn_write(client, &rec_char, sizeof(char), NETCONN_COPY); 
    	}
    	xQueueReset(*incoming_queue);
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

const int ESC=27;

void rx_task(void * pvParameters) {
	uint8_t start = 0;
	xQueueHandle * incoming_queue = (xQueueHandle *)pvParameters;
	packet * msg;
	msg = (packet *) malloc(sizeof(packet));
	uint32_t packet_size = 0;
	char last_char = NULL;
	char last_last_char = 0xFF;
	while(1) {
		char rec_char = uart_getc(0);
		//printf("%c", rec_char);
		if (rec_char == ESC) {
			printf("---packet size---: %d\n", packet_size);
			start = 0;
			if( packet_size != 0) {
        		msg->written = packet_size;
        		packet_size = 0;
        		xQueueSend(*incoming_queue, &msg, 1000/portTICK_RATE_MS);
        		//xQueueSend(*incoming_queue, &msg, 1000/portTICK_RATE_MS);
        		free(msg);
        		msg = (packet *) malloc(sizeof(packet));
        		printf("---Sent partial packet: %d\n", msg->written);
            }
		}

        if (rec_char == ATT_CHAR && last_char == ATT_CHAR && last_last_char == ATT_CHAR) {
        	printf("---Starting packet save---\n");
        	start = 1;
        }

        if (start) {
        	if(packet_size < PACKET_SIZE) {
        		msg->data[packet_size] = rec_char;
        		packet_size++;
        	}
        	else{
        		printf("---Sending full packet---\n");
        		msg->written = packet_size;
        		packet_size = 0;
        		xQueueSend(*incoming_queue, &msg, 1000/portTICK_RATE_MS);
        		free(msg);
        		msg = (packet *) malloc(sizeof(packet));
        	}
        }
        //vTaskDelay(1000/portTICK_RATE_MS);
        //printf("%c:%c:%c\n", rec_char, last_char, last_last_char);
        last_last_char = last_char;
        last_char = rec_char;

	}
}


void configure_wifi() {
	
	//sdk_wifi_set_opmode(SOFTAP_MODE);

	//struct sdk_softap_config *config = (struct sdk_softap_config *)calloc(1, sizeof(struct sdk_softap_config));

  	//sdk_wifi_softap_get_config(config);
  	//sprintf(config->ssid, (uint8_t) SSID);
  	//sprintf(config->password, (uint8_t) PASSWORD);
  	//config->authmode = AUTH_WPA_WPA2_PSK;
  	//config->ssid_len =  (uint8_t) strlen(SSID);
  	//config->max_connection = 4;


    struct sdk_softap_config ap_config = {
        .ssid = SSID,
        .ssid_hidden = 0,
        .channel = 1,
        .ssid_len = strlen(SSID),
        .authmode = AUTH_OPEN,
        .password = PASSWORD,
        .max_connection = (uint8_t) 3,
        .beacon_interval = 100,
    };

    sdk_wifi_softap_set_config(&ap_config);
    //ree(config);

    dhcpserver_stop();
    struct ip_info ap_ip;
    IP4_ADDR(&ap_ip.ip, 192, 168, 1, 1);
    IP4_ADDR(&ap_ip.gw, 192, 168, 1, 1);
    IP4_ADDR(&ap_ip.netmask, 255, 255, 255, 0);
    sdk_wifi_set_ip_info(1, &ap_ip);

    ip_addr_t first_client_ip;
    IP4_ADDR(&first_client_ip, 192, 168, 1, 2);
    dhcpserver_start(&first_client_ip, 2);
}

static xQueueHandle incoming_queue;


void user_init(void)
{
	//system_set_os_print(0);
	uart_set_baud(0, 115200);
	printf("---Configuring WiFi---\n");
	configure_wifi();
	while(!sdk_wifi_set_opmode(SOFTAP_MODE)){
    	printf("Setting up wifi...");
    };
    printf("---SDK version:%s---\n", sdk_system_get_sdk_version());
    incoming_queue = xQueueCreate( 5, sizeof(packet *));
    xTaskCreate(rx_task, (signed char *) "rx_task", 2048, &incoming_queue, 2, NULL);
    xTaskCreate(wifi_80_task, (signed char *) "wifi_80_task", 4096,&incoming_queue, 10, NULL);
    printf("---User init complete!---\n");
}

