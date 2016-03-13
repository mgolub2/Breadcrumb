"""
Simulate the chain parsing and replying to a http request
"""

import serial
import socket
import re
import traceback
import sys
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
        data_array = data.split(',')
        '''
        data_array = data.split(',')
        #print(data_array)
        #Index 0 is return IP, index 1 is return port, and index 3 is the http info
        #print("DATA:   " , data_array[2])
        host_match = re.search('Host: (\S+)\\r\\n', data_array[2])
        if host_match:
            host = host_match.group(1)
            print(host)
            try:
                host, port = host.split(':')
            except ValueError:
                port = 80
            print(host, port)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(4)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.connect((host, int(port)))
            print("***Proxy connect***")
            s.send(data_array[2].encode('utf-8'))
            print("***Proxy send****")
            rx_data = s.recv(4096)
            s.shutdown(1)
            s.close()
            #print("*****************************Recieved: \n", rx_data, "\n")
            '''
            #serial_socket.write('==={0}==='.format(rx_data).encode('ascii'))
        with open('hackaday.txt', 'r') as d:
            serial_socket.write('==={0},{1},{2}==='.format(data_array[0], data_array[1], d.read()).encode('ascii'))
    except Exception as e:
        print(e)
        traceback.print_exc(file=sys.stdout)


if __name__ == '__main__':
    loop()
