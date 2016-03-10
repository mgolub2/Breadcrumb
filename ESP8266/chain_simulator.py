"""
Simulate the chain parsing and replying to a http request
"""


import serial

PORT = '/dev/cu.usbserial-FTZ29WSV'
BAUD = 115200


def loop():
	with serial.Serial(PORT, BAUD) as serial_socket:
		data = ""
		while True:
			bytes_to_read = serial_socket.inWaiting()
			new_data = serial_socket.read(bytes_to_read)
			if new_data:
				data+=new_data
				print(new_data)

loop()