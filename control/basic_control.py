#!/usr/bin/env python

import serial

def main():
	# Open the serial port.
	ser = serial.Serial('/dev/ttyACM0')
	print(ser.name)

	# Play sound to indicate script start.
	ser.write(b'\rPlaySound 1\r')

	# Setup test mode.
	ser.write(b'TestMode On\r')
	ser.write(b'SetMotor LWheelEnable\r')
	ser.write(b'SetMotor RWheelEnable\r')

	# Take basic commands.
	cmd = ''
	while cmd is not 'q':
		cmd = raw_input('>>>')
		if cmd is 'w':
			ser.write(b'SetMotor 100 100 200 0\r')
		elif cmd is 'ww':
			ser.write(b'SetMotor 1000 1000 100 0\r')
		elif cmd is 's':
			ser.write(b'SetMotor -100 -100 200 0\r')
		elif cmd is 'ss':
			ser.write(b'SetMotor -1000 -1000 100 0\r')
		elif cmd is 'a':
			ser.write(b'SetMotor -100 100 200 0\r')
		elif cmd is 'aa':
			ser.write(b'SetMotor -1000 1000 100 0\r')
		elif cmd is 'd':
			ser.write(b'SetMotor 100 -100 200 0\r')
		elif cmd is 'dd':
			ser.write(b'SetMotor 1000 -1000 100 0\r')

	# We're done.
	ser.write(b'TestMode Off\r')
	ser.close()

if __name__ == '__main__':
	main()
