from sys import argv
from os.path import exists
import serial
import pymu

#Define serial port
port = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=3.0)

# Initialize IMU
pymu.startup()

#IMU Commands
#pymu.get_pitch()
#pymu.get_roll()
#pymu.get_yaw()

while True:
	rcv = port.read(10)

	if rcv in 'pitch':
		port.write(repr(pymu.get_pitch()))

	if rcv in 'roll':
		port.write(repr(pymu.get_roll()))

	if rcv in 'yaw':
		port.write(repr(pymu.get_yaw()))