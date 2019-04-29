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
        self.Climbcrc=ClimbRobotCRC()
        try:
            self.ser = serial.Serial(port=self.PORT, baudrate=self.Baudrate, bytesize=8, parity='O', stopbits=1,timeout=0.5, xonxoff=0)
        except:
            print("Please check Usb port----")
    def get_serial_port(self):
        a,b=commands.getstatusoutput('python -m serial.tools.list_ports')
        print b
    def Read_info_from_driver(self):
        Modbus_Enable_info=[0x01,0x03,0x00,0x00,0x00,0x01]

    def Opreating_Info_CRC(self):
        pass
    def Position_abs_control(self):
        pass
    def Position_incremental_control(self):
        pass
    def Velocity_control(self):
        pass
    def Pulse_32bits_change(self,pusle_value):
        if pusle_value<0:
            return hex(pusle_value&0xffffffff)
        else:
            return hex(pusle_value)
    def Pulse_16bits_change(self,pusle_value):
        if pusle_value<0:
            return hex(pusle_value&0xffff)
        else:
            return hex(pusle_value)
    def Send_message_to_port(self,message):
        """
        :param message: String "010600000001480A" or "01 06 00 00 00 01 48 0A"
        :return:
        """
        message_bytes = message.replace(" ",'').decode('hex')
        # print str(hex(message_bytes))
        self.ser.write(message_bytes)
        time.sleep(0.01)

    def Enable_Modbus_serial(self):

        Modbus_Enable='01 06 00 00 00 01 48 0A'
        self.Send_message_to_port(Modbus_Enable)

    def Disable_Modbus_serial(self):
        Modbus_Disable=[0x01,0x06,0x00,0x00,0x00]#'01 06 00 00 00 00 48 0A'
        Modbus_Disable=self.Climbcrc.Combining_CRC_and_info(Modbus_Disable)
        self.Send_message_to_port(Modbus_Disable)

def main():
    PORT = "/dev/ttyUSB0"
    baudrate=19200
    climb_robot=ClimbRobot(PORT,baudrate)
    climb_robot.get_serial_port()
    # climb_robot.Disable_Modbus_serial()
    #
    print climb_robot.Pulse_16bits_change(-100)

if __name__ == "__main__":
    main()