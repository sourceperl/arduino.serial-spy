#!/usr/bin/python

import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
  port='/dev/ttyUSB0',
  baudrate=9600,
#    parity=serial.PARITY_ODD,
#    stopbits=serial.STOPBITS_TWO,
#    bytesize=serial.SEVENBITS
)

i = 0

while 1 :
#  ser.write(time.asctime()+"\r\n")
  for i in range(0,256):
    ser.write(chr(i))
    time.sleep(0.01)
