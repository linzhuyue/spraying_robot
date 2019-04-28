#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:3----->roation
id:2------>Upper and lower climbing pole
"""

import sys
import binascii
import inspect
import serial
import commands
import time
from Climb_RCR_Check import *
class ClimbRobot:
    def __init__(self,PORT,Baudrate):
        self.PORT=PORT
        self.Baudrate=Baudrate
        try:
            self.ser = serial.Serial(port=self.PORT, baudrate=self.Baudrate, bytesize=8, parity='O', stopbits=1, xonxoff=0)
        except:
            print("Please check Usb port")
    def get_serial_port(self):
        a,b=commands.getstatusoutput('python -m serial.tools.list_ports')
        print b,type(b)
    def 01
    def Position_abs_control(self):
        pass
    def Position_incremental_control(self):
        pass
    def Velocity_control(self):
        pass
    def Pulse_hex_change(self,pusle_value):
        if pusle_value<0:
            return hex(pusle_value&0xffffffff)
        else:
            return hex(pusle_value)
    def Open_serial(self):
        if self.ser.open():
            print "Open Ok"
            hex_string='01 06 00 00 00 01 48 0A'
            print '',binascii.hexlify(hex_string)
            self.ser.write(b'hello')
            time.sleep(0.1)
        else:
            print "Please check your USB port -----"



#PORT = '/dev/ttyp5'
def main():
    PORT = "/dev/ttyUSB0"
    baudrate=19200
    climb_robot=ClimbRobot(PORT,baudrate)
    climb_robot.get_serial_port()
    hex_string='01 06 00 00 00 01 48 0A'
    hh='01 06 00 02 03 E8 28 B4'



if __name__ == "__main__":
    main()