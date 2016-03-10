"""
Simulate the chain parsing and replying to a http request
"""

import serial
import socket
import re
# PORT = '/dev/cu.usbserial-FTZ29WSV'
PORT = 'COM3'
BAUD = 115200


def loop():
    with serial.Serial(PORT, BAUD) as serial_socket:
        pound_count = 0
        data = ''
        while True:
            #bytes_to_read = serial_socket.inWaiting()
            new_data = serial_socket.read()#read(bytes_to_read)
            try:
                decode_data = new_data.decode('utf-8')
                if decode_data:
                    if decode_data == '#':
                        pound_count += 1
                    if pound_count >= 3 and decode_data != '#':
                        data += decode_data
                    if pound_count == 6:
                        #print(data)
                        parse(data, serial_socket)
                        pound_count = 0
                        data = ''
                    #print(decode_data, end="")
            except UnicodeDecodeError:
                    pass

def parse(data, serial_socket):
    try:
        data_array = data.split(',')
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
            s.connect((host, int(port)))
            s.send(data_array[2].encode())
            rx_data = s.recv(4096)
            serial_socket.write('==={0},{1},{2}==='.format(data_array[0], data_array[1], data_array[2]))
    except Exception as e:
        print(e)

loop()
