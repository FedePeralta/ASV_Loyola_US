import serial

SERIAL_BOARD = 'ARDUINO_UNO'

class SerialBoard():

	def __init__(self, ser_path = 'serial_path', baudrate = 9600, timeout = None):
		""" Create the serial object """

		""" Path to the serial. COMx in Windows, /tty/USBx in Linux Based"""
		self.serial_path = ser_path
		""" Create the serial object"""
		self.serial_connection = serial.Serial(ser_path, baudrate = baudrate, timeout = timeout)


	def write_gpio(self, value):

		assert value in ['HIGH', 'LOW'], "The only values admitted are HIGH and LOW"

		if value == 'HIGH':
			self.serial_connection.write('H'.encode('UTF-8'))
		else:
			self.serial_connection.write('L'.encode('UTF-8'))

	def __del__(self):

		print("SerialBoard disconnected!")
		self.serial_connection.close()