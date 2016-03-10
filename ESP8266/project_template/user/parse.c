//
// Created by mgolub2 on 3/10/2016.
//

#include "parse.h"
#include "lwip/sockets.h"
#include <string.h>

int parse(char * tcp_data, uint16 data_index) {
    tcp_data[data_index+1] = '\0'; //make sure we end the string, space already accounted for.
    char * token;
    char * connection_data[3];
    uint8 con_index = 0;
    while((token = strstep(tcp_data, ',')) != NULL && con_index < 3) {
        connection_data[con_index] = token;
        con_index++;
    }
    char * endptr;
    incoming_packet packet = malloc(sizeof(incoming_packet));
    incoming_packet->tcp_data_len = strlen(connection_data[2]);
    incoming_packet->tcp_data = connection_data[2]; //returned from website, hopefully
    incoming_packet->addr = inet_addr(connection_data[0]); //convert ip string to int
    incoming_packet->port = strtoimax(connection_data[1], &endptr, 10); //convert str to int, base 10
    return 0;
}