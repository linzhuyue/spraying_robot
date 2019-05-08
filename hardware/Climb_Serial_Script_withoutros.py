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
# import commands
import subprocess
import time
from Climb_RCR_Check import *
from textwrap import wrap
class ClimbRobot:
    def __init__(self,PORT,Baudrate):
        self.readstringlength=25
        self.PORT=PORT
        self.Baudrate=Baudrate
        self.Climbcrc=ClimbRobotCRC()
        try:
            self.ser = serial.Serial(port=self.PORT, baudrate=self.Baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=1,timeout=0.3, xonxoff=0,rtscts=False,dsrdtr=False)
        except:
            print("Please check Usb port----")
    def get_serial_port(self):
        a,b=subprocess.getstatusoutput('python -m serial.tools.list_ports')
        print(b)
        return b
    def Send_PulsePulse_to_driver(self,deviceaddr,function_code,firstregistoraddr_high16,firstregistoraddr_low16,PU8_15,PU0_7,PU24_31,PU16_23,datalen=4,registornum_high16=0,registornum_low16=2):
        """
        :param deviceaddr: just 0x10,you can use decimal or hexadecimal
        :param function_code:
        :param firstregistoraddr_high16:
        :param firstregistoraddr_low16:
        :param PU8_15:
        :param PU0_7:
        :param PU24_31:
        :param PU16_23:
        :param datalen:
        :param registornum_high16:
        :param registornum_low16:
        :return:read registor buffer information
        """
        sendstring=self.Climbcrc.Combining_CRC_and_info([deviceaddr,function_code,firstregistoraddr_high16,firstregistoraddr_low16,registornum_high16,registornum_low16,datalen,PU8_15,PU0_7,PU24_31,PU16_23])
        readbuffer=self.Send_message_to_port(sendstring)
        print("---port read buffer info---", readbuffer)
        return readbuffer
    def Send_Code_to_driver(self,deviceaddr,function_code,firstregistoraddr_high16,firstregistoraddr_low16,data_high16,data_low16):
        """

        :param deviceaddr: int ,here you can use 0x06, you can also use Decimal
        :param function_code:int
        :param firstregistoraddr_high16:int
        :param firstregistoraddr_low16:int
        :param data_high16:int
        :param data_low16:int
        :return: read registor buffer information
        """
        sendstring=self.Climbcrc.Combining_CRC_and_info([deviceaddr,function_code,firstregistoraddr_high16,firstregistoraddr_low16,data_high16,data_low16])
        readbuffer=self.Send_message_to_port(sendstring)
        print("---port read buffer info---", readbuffer)
        return readbuffer
    def Read_info_from_driver(self,function_code,deviceaddr,registoraddr_high16,registoraddr_low16,firstregistoraddr_high16=0,firstregistoraddr_low16=0):
        """
        :param function_code: int here just 0x03 you can also use 3
        :param deviceaddr: int
        :param registoraddr_high16: int
        :param registoraddr_low16: int
        [0x01,0x03,0x00,0x00,0x00,0x01]
        :return:int list
        """
        sendstring=self.Climbcrc.Combining_CRC_and_info([deviceaddr,function_code,firstregistoraddr_high16,firstregistoraddr_low16,registoraddr_high16,registoraddr_low16])
        # Modbus_Enable_info=[0x01,0x03,0x00,0x00,0x00,0x01]
        readbuffer=self.Send_message_to_port(sendstring)
        print("---port read buffer info---", readbuffer)
        return readbuffer
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
        :return: int list
        """
        message_bytes = message.replace(" ",'').decode('hex')
        # print str(hex(message_bytes))
        self.ser.write(message_bytes)
        self.ser.flushInput()
        self.ser.flushOutput()
        time.sleep(0.01)
        strt = self.ser.read(self.readstringlength).encode('hex')
        readbuffer=[int(int(i, 16)) for i in wrap(strt, 2)]
        return readbuffer
    def Enable_Modbus_serial(self,deviceaddr):

        Modbus_Enable=[deviceaddr,6,0,0,0,1]#'01 06 00 00 00 01 48 0A'
        Modbus_Disable=self.Climbcrc.Combining_CRC_and_info(Modbus_Enable)
        readbuffer,strt=self.Send_message_to_port(Modbus_Disable)
        print("---port read buffer info---",readbuffer)

    def Disable_Modbus_serial(self,deviceaddr):
        Modbus_Disable=[deviceaddr,0x06,0x00,0x00,0x00]#'01 06 00 00 00 00 48 0A'
        Modbus_Disable=self.Climbcrc.Combining_CRC_and_info(Modbus_Disable)
        readbuffer,strt=self.Send_message_to_port(Modbus_Disable)
        print("---port read buffer info---",readbuffer)

def main():
    PORT = "/dev/ttyUSB0"
    baudrate=19200
    climb_robot=ClimbRobot(PORT,baudrate)
    climb_robot.get_serial_port()
    # climb_robot.Disable_Modbus_serial()
    #
    print(climb_robot.Pulse_16bits_change(-100))

if __name__ == "__main__":
    main()