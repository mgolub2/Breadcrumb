/*
Uber simple dns server for the ESP8266. Will respond with DNS_IP to all queries.n

Author: Max Golub

Based off: http://www.samiam.org/software/microdns-20100805.c
License:
Copyright (c) 2009-2010 Sam Trenholme
 *
 * TERMS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * This software is provided 'as is' with no guarantees of correctness or
 * fitness for purpose.

*/

#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "esp_common.h"

/*
Set our ip address in the dns header
*/
uint32_t set_ip(uint32_t ip, char header[17]) {
    //check the endiannes of this, might need to be little endian?
	header[12] = (ip & 0xff000000) >> 24;
    header[13] = (ip & 0x00ff0000) >> 16;
    header[14] = (ip & 0x0000ff00) >>  8;
    header[15] = (ip & 0x000000ff);
}

/*
Setup port 53 with a socket, and return that
*/
int get_port(char ip[], char header[17], struct sockaddr_in *dns_udp) {
	int sock;
	int len_inet;

	sock = socket(AF_INET, SOCK_DGRAM, 0);
	//printf("Created socket\n");

	if(sock == -1) {
		printf("FAILURE: Could not create DNS socket\n");
		//idk what'll happen now. YOLO
	}

	uint32_t byte_ip = ntohl(inet_addr(ip));

	memset(dns_udp,0,sizeof(struct sockaddr_in));
    dns_udp->sin_family = AF_INET;
    dns_udp->sin_port = PP_HTONS(53);
    dns_udp->sin_addr.s_addr = inet_addr(ip);

    set_ip(byte_ip, header);
    len_inet = sizeof(struct sockaddr_in);

    if(bind(sock,(struct sockaddr *)dns_udp,len_inet) == -1) {
        printf("Bind error!\n");
    }

    //printf("Returning socket\n");
    return sock;
}

/*
Task for FreeRTOS to call and serve tasty DNS on.
*/
void ICACHE_FLASH_ATTR dns_server_task(void *pvParameters) {
	printf("Starting DNS server...");
	char header[17] = "\xc0\x0c\x00\x01\x00\x01\x00\x00\x00\x00\x00\x04\x7f\x7f\x7f\x7f";
	int a, len_inet;
    int sock;
    char in[512];
    socklen_t foo = sizeof(in);
    struct sockaddr_in dns_udp;
    uint32_t ip = 0; /* 0.0.0.0; default bind IP */
    int leni = sizeof(struct sockaddr);
    sock = get_port("192.168.1.1", header, &dns_udp);
    for(;;) { //same as while 1
    	/* Get data from UDP port 53 */
    	//printf("Wait for DNS queries");
        len_inet = recvfrom(sock,in,255,0,(struct sockaddr *)&dns_udp,
                &foo);
        /* Roy Arends check: We only answer questions */
        if(len_inet < 3 || (in[2] & 0x80) != 0x00) {
                continue;
        }

        /* Prepare the reply */
        if(len_inet > 12) {
        	    //printf("Prep DNS reply...");
                /* Make this an answer */
                in[2] |= 0x80;
                if(in[11] == 0) { /* EDNS not supported */
                        /* We add an additional answer */
                        in[7]++;
                } else {
                        in[3] &= 0xf0; in[3] |= 4; /* NOTIMPL */
                }
        }
        if(in[11] == 0) { /* Again, EDNS not supported */
                for(a=0;a<16;a++) {
                        in[len_inet + a] = header[a];
                }
        }
        //printf("Send DNS Reply");
        /* Send the reply */
        sendto(sock,in,len_inet + 16,0, (struct sockaddr *)&dns_udp, leni);
    }
}
