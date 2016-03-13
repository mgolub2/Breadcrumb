//
// Created by mgolub2 on 3/10/2016.
//

#ifndef ESP8266_PARSE_H
#define ESP8266_PARSE_H

#include "esp_common.h"
#include "uart.h"

typedef struct {
    //uint16 tcp_data_len;
    char tcp_data[PACKET_SIZE];
    uint16 port;
    unsigned long addr;
} incoming_packet;

void parse(char * tcp_data, uint16 data_index, incoming_packet * packet);

#endif //ESP8266_PARSE_H
