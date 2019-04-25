#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:2----->roation
id:3------>Upper and lower climbing pole
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
    def Open_serial(self):
        if self.ser.open():
            print "Open Ok"
            hex_string='01 06 00 00 00 01 48 0A'
            print '',binascii.hexlify(hex_string)
            self.ser.write(b'hello')
        else:
            print "Please check your USB port -----"
    def send_command(self,cmd_name, cmd_string):
        print ("\ncmd_name:", cmd_name)
        print ("cmd_string:", cmd_string)
        cmd_bytes = bytearray.fromhex(cmd_string)
        # print cmd_bytes
        for cmd_byte in cmd_bytes:
            hex_byte = ("{0:02x}".format(cmd_byte))

            print (hex_byte)
            self.ser.write(bytearray.fromhex(hex_byte))
            # time.sleep(.100)

        # wait an extra 3 seconds for DISP_ON_CMD
        if cmd_name == "DISP_ON_CMD":
            time.sleep(5.0)
        response = self.ser.read(32)
        print ("response:", binascii.hexlify(bytearray(response)))
        self.ser.close()
        return


#PORT = '/dev/ttyp5'
def main():
    PORT = "/dev/ttyUSB0"
    baudrate=19200
    climb_robot=ClimbRobot(PORT,baudrate)
    climb_robot.get_serial_port()
    hex_string='01 06 00 00 00 01 48 0A'
    hh='01 06 00 02 03 E8 28 B4'
    climb_robot.send_command('HEART_BEAT_CMD',hex_string.replace(' ',''))
    climb_robot.send_command('HEART_BEAT_CMD',hh)

    # kk=hex_string.replace(' ','')
    # print kk,kk.decode('hex')
    # Code to put processor into factory mode.
    # comm_init='c4c4'
    # Here's the list of command strings, captured as tuples.
    # The 16-bit Data and Msg CRCs are calculated via gen_crc.exe.
    # heart_beat_cmd=      ("HEART_BEAT_CMD",      'a5a50a00010000003e1b')
    # print (hex_string.replace(' ','')).decode('hex')
    # # print '',unhexlify(hex_string.strip(' '))
    # print climb_robot.ser.write((hex_string.replace(' ','')).decode('hex'))
    # # print climb_robot.ser,type(climb_robot.ser)
    # # climb_robot.Open_serial()
if __name__ == "__main__":
    main()