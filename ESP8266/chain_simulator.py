"""
Simulate the chain parsing and replying to a http request
"""

import serial
import socket
import re
import traceback
import sys
import subprocess

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
                    if decode_data == '#':
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
            rx_data = subprocess.check_output(['curl', '-i', host])
            serial_socket.write('==={0}==='.format(rx_data).encode('ascii'))
        #with open('hackaday.txt', 'r') as d:
        #    serial_socket.write('==={0}==='.format(d.read()).encode('utf-8'))
    except Exception as e:
        print(e)
        traceback.print_exc(file=sys.stdout)


if __name__ == '__main__':
    loop()
