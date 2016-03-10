//
// Created by mgolub2 on 3/10/2016.
//

#ifndef ESP8266_PARSE_H
#define ESP8266_PARSE_H

#include "lwip/sockets.h"

int parse(char * tcp_data, uint16 data_index);

typedef struct {
    uint16 struct_data_len ;
    char * tcp_data;
    uint16 port;
    unsigned long addr;
} incoming_packet;

#endif //ESP8266_PARSE_H
