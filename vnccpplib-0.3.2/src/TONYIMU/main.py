from sys import argv
from os.path import exists
import serial
from subprocess import call

port = serial.Serial("/dev/ttyAMA0", baudrate=115200, timeout=3.0)

while True:
	rcv = port.read(10)

	if rcv in 'pitch':
		call("./saveIMUdata")
		in_file = open('imudata.txt')
		lines = in_file.readlines()
		port.write(repr(lines[2]))
		in_file.close()