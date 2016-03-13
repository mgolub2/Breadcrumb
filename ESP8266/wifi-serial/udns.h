/*
Header file for the uber simple DNS server

Author: Max Golubs
*/

#ifndef __UDNS_H__
#define __UDNS_H__

uint32_t set_ip(uint32_t ip, char header[17]);
int get_port(char ip[], char header[17], struct sockaddr_in *dns_udp); 
void ICACHE_FLASH_ATTR dns_server_task(void *pvParameters);

#endif