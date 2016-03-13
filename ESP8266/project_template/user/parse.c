//
// Created by mgolub2 on 3/10/2016.
//

#include "parse.h"
#include <string.h>
#include "esp_common.h"
#include "lwip/sockets.h"

void parse(char * tcp_data, uint16 data_index, incoming_packet * packet) {
    tcp_data[data_index+1] = '\0'; //make sure we end the string, space already accounted for.
    char * token;
    char * connection_data[3];
    uint8 con_index = 0;
    while((token = (char *) strsep(&tcp_data, ",")) != NULL && con_index < 3) {
        connection_data[con_index] = token;
        con_index++;
    }
    char * endptr;
    //incoming_packet * packet;
    //packet = (incoming_packet *) malloc(sizeof(incoming_packet));
    //packet->tcp_data_len = sizeof(connection_data[2]);
    strncpy(packet->tcp_data, connection_data[2], strlen(connection_data[2])); //returned from website, hopefully
    packet->addr = inet_addr(connection_data[0]); //convert ip string to int
    packet->port = strtol(connection_data[1], &endptr, 10); //convert str to int, base 10
    //return packet;
}
