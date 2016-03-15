"""
Simulate the chain parsing and replying to a http request
"""

import serial
import socket
import re
import traceback
import sys
import subprocess
import time

PORT = '/dev/cu.usbserial-FTZ29WSV'
#PORT = 'COM3'
BAUD = 115200


def loop():
    with serial.Serial(PORT, BAUD) as serial_socket:
        pound_count = 0
        data = ''
        while True:
            #bytes_to_read = serial_socket.inWaiting()
            new_data = serial_socket.read(1)#read(bytes_to_read)
            #print(len(new_data))
            try:
                decode_data = new_data.decode('ascii')
                print(decode_data, end="")
                if decode_data:
                    if decode_data == '\a':
                        pound_count += 1
                        print(pound_count)
                        if pound_count == 6:
                            print("***Parsing data!!!****")
                            parse(data, serial_socket)
                            pound_count = 0
                            data = ''
                    else:
                        if pound_count >= 3 < 6:
                            data += decode_data
                        else:
                            pound_count = 0
                    #print(decode_data, end="")
            except UnicodeDecodeError:
                    pass

def parse(data, serial_socket):
    try:
        host_match = re.search('Host: (\S+)\\r\\n', data)
        if host_match:
            host = host_match.group(1)
            print(host)
            try:
                host, port = host.split()
            except ValueError:
                port = 80
            if host == "192.168.1.1:8080":
                with open('hackaday.txt', 'r') as d:
                    data = d.read(100)
                    serial_socket.write('\b\b\b'.encode('utf-8'))
                    #serial_socket.flushOutput()
                    while data:
                        serial_socket.write(data.encode('utf-8'))
                        #serial_socket.flushOutput()
                        data = d.read(100)
                        if chr(27).encode() in data.encode():
                            print("OH SHIT")
                        print(len(data))
                        time.sleep(.01)
                        #print(data)
                    serial_socket.write(chr(27).encode())
                    #serial_socket.flushOutput()
                    #serial_socket.write('\b\b\b{0}\b\b\b'.format(d.read()).encode('utf-8'))
            else:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((host, int(port)))
                totalsent = 0
                while totalsent < len(data):
                    sent = s.send(data[totalsent:])
                    if sent == 0:
                        raise RuntimeError("socket connection broken")
                    totalsent = totalsent + sent
                result = s.recv(10000)
                if result:
                    serial_socket.write('\b\b\b'.encode('utf-8'))
                while (len(result) > 0):
                    serial_socket.write(result.encode())
                    result = s.recv(10000)
                serial_socket.write(chr(27).encode())
                s.close()
    except Exception as e:
        print(e)
        traceback.print_exc(file=sys.stdout)


if __name__ == '__main__':
    loop()
