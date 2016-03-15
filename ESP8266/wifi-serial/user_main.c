/* esp-open-rtos based project to print TCP data to the serial port, then 
 * send responses from the breadcrumb chain to the client device. 
 *
 * Author: Maximilian Golub
 *
 * Uses tasks to keep everything working together nicely. UART is _not_ intterupt
 * based.  
 * 
 * Uses the lwip netconn library, since sockets seem to make the ESP8266 sad.
 * Lots of things make the ESP8266 sad, like to many clients, requests, dirty looks,
 * and power that is not conditioned by the highest end audophile equitment. 
 */

//So many incluuudddesss
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

//So many defffinnesss
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

//Struct used to pass data from uart rx task to wifi/netconn task
typedef struct {
	char data[PACKET_SIZE];
	uint32_t written; //amount of space actually used in the data array
} packet;

//prototypes for functions
void wifi_80_task(void *pvParameters);
void rx_task(void *pvParameters);
void configure_wifi();
void user_init(void);
void send_att_char();
void heap_mon(void * pvParameters);

//Helper function to send three of the attention chars
void send_att_char() {
	uint8_t char_inc;
	for (char_inc = 0; char_inc < NUM_ATT_CHAR; char_inc++) {
        putchar(GSM_ATT_CHAR);
    }
}

/*
 * Handles the netconn connection setup and handling of clients
 * Binds and listens on port 8080 by default. Waits to recieve 
 * data from a TCP client.  
 */
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
  	//Loop forever!	
	while(1) {
		struct netconn *client = NULL;
    	err_t err = netconn_accept(nc, &client); //Do we have friends
    	printf("---Ack connection...---\n"); //friends!
    	if ( err != ERR_OK ) { //MEAN friends
      		if(client) {
      			printf("---ERROR!!:: %d---\n", err);
				netconn_delete(client);
      		}
      		continue;
    	}
    	struct netbuf *netbuf;
    	send_att_char();
    	netconn_set_recvtimeout(client, 1000/portTICK_RATE_MS);
    	//Get data from our friends/clients
    	while((err = netconn_recv(client, &netbuf)) == ERR_OK) {
    		uint16_t len = netbuf_len(netbuf);
	        uint16_t offset;
	        for(offset = 0; offset<len; offset++) {
	        	putchar(netbuf_read_u8(netbuf, offset));
        	}
    	}
    	putchar(ATT_CHAR);
    	packet * msg;
    	printf("---%lu left in queue---\n", uxQueueMessagesWaiting(*incoming_queue));
    	//Block on waiting for data from down the chain, sent to us from UART RX
    	while(xQueueReceive( *incoming_queue, &msg, SOCKET_WAIT_TIME/portTICK_RATE_MS )) {
    		printf("---Packet size: %d---\n", msg->written);
    		netconn_write(client, msg->data, msg->written, NETCONN_COPY);
    	}
    	xQueueReset(*incoming_queue);
		netconn_close(client);
    	while((err = netconn_delete(client)) != ERR_OK) {
    		printf("---Can't close connection: %d---\n", err);
    		vTaskDelay(1000/portTICK_RATE_MS);
    	}
    	netbuf_delete(netbuf);
	}
}

const int ESC=27;

//Print out heap stats every now and then. 
void heap_mon(void * pvParameters) {
	while(1) {
		printf("---Free heap %d bytes---\n", (int)xPortGetFreeHeapSize());
		vTaskDelay(10000/portTICK_RATE_MS);
	}
}

/*
 * Wait around for the UART to get data so we can check it for chars
 * indicating an incoming tcp datastream from down the chain. 
 * Sends that data to the wifi_80 task. 
 */
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
        		xQueueSend(*incoming_queue, &msg, 7000/portTICK_RATE_MS);
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
        //Set the variables we use to check for three of the att char in a row
        last_last_char = last_char;
        last_char = rec_char;

	}
}

/*
 * Setup and configure the esp8266 WIFI how we like it
 */
void configure_wifi() {
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

//queue used to pass data from uart_rx -> wifi_80_task
static xQueueHandle s_incoming_queue;

/*
 * Start everything up!
 */
void user_init(void)
{
	uart_set_baud(0, 115200);
	printf("---Configuring WiFi---\n");
	configure_wifi();
	while(!sdk_wifi_set_opmode(SOFTAP_MODE)){
    	printf("Setting up wifi...");
    };
    printf("---SDK version:%s---\n", sdk_system_get_sdk_version());
    incoming_queue = xQueueCreate( 4, sizeof(packet *));
    xTaskCreate(heap_mon, (signed char *) "heap_mon", 256, NULL, 2, NULL);
    xTaskCreate(rx_task, (signed char *) "rx_task", 2048, &s_incoming_queue, 4, NULL);
    xTaskCreate(wifi_80_task, (signed char *) "wifi_80_task", 4096,&s_incoming_queue, 13, NULL);
    printf("---User init complete!---\n");
}

