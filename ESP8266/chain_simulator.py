#!/usr/local/bin/python3.5
"""
Author: Maximilian Golub

Simulate the chain parsing and replying to a http request to test
the ESP8266
"""

import serial
import socket
import re
import traceback
import sys
import subprocess
import time

PORT = '/dev/cu.usbserial-FTZ29WSV' #OSX
#PORT = 'COM3' # If on windows
BAUD = 115200


def loop():
    """
    Loop that runs forever, reading from the
    serial connection waiting for special sequences indicating
    TCP data.
    :return:
    """
    with serial.Serial(PORT, BAUD) as serial_socket:
        pound_count = 0
        data = ''
        start = 0
        while True:
            #Read a character at a time.
            #Yes this is awful and terrible
            new_data = serial_socket.read(1)
            try:
                decode_data = new_data.decode('ascii')
                #print(decode_data, end="")

                if decode_data:
                    if decode_data == '\a':
                        pound_count += 1
                        if pound_count >= 3:
                            start = 1
                        print(pound_count)
                    else:
                        pound_count = 0
                    if decode_data == '\b':
                        print("***Parsing data!!!****")
                        start = 0
                        print(data)
                        parse(data, serial_socket)
                        pound_count = 0
                        data = ''
                    else:
                        if start:
                            data += decode_data
            except UnicodeDecodeError:
                    pass


def parse(data, serial_socket):
    """
    Parse the data coming over the serial connection. The data should
    be the GET/POST whatever request from the Wifi device attached to the
    ESP8266. Looks for the Host header, trys to get the host+port with regex.
    :param data:
    :param serial_socket:
    :return:
    """
    try:
        host_match = re.search('Host: (\S+)\\r\\n', data)
        if host_match:
            host = host_match.group(1)
            #print(host)
            try:
                host, port = host.split()
            except ValueError:
                port = 80
            if host == "192.168.1.1:8080": # Special case to test basic functionality level.
                with open('hackaday.txt', 'r') as d:
                    data = d.read(100)
                    serial_socket.write('\b\b\b'.encode('utf-8'))
                    while data:
                        serial_socket.write(data.encode('utf-8'))
                        data = d.read(100)
                        if chr(27).encode() in data.encode():
                            print("OH SHIT")
                        time.sleep(.01)
                    serial_socket.write(chr(27).encode())
            else: #Connect a socket as a client, then return that over the uart.
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                s.connect((host, int(port)))
                totalsent = 0
                while totalsent < len(data): #Send all of our data
                    sent = s.send(data[totalsent:].encode())
                    if sent == 0:
                        raise RuntimeError("socket connection broken")
                    totalsent = totalsent + sent
                result = s.recv(100) #Recieve data in 100 byte chunks, just like the special case.
                if result:
                    serial_socket.write('\b\b\b'.encode('utf-8')) #Write special start code sequence
                while (len(result) > 0):
                    serial_socket.write(result)
                    time.sleep(.01) #Keep the ESP8266 from sploding
                    result = s.recv(100)
                serial_socket.write(chr(27).encode())
                s.close()
    except Exception as e:
        print(e)
        traceback.print_exc(file=sys.stdout)


if __name__ == '__main__':
    loop()
