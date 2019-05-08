
#!/usr/bin/env python
# -*- coding: utf_8 -*-
import rospy
import sys
import binascii
import inspect
import serial
import commands
import time
from Climb_RCR_Check import *
from textwrap import wrap
from std_msgs.msg import String

class ClimbRobot:
    def __init__(self,PORT,Baudrate,nodename):
        self.readstringlength=25
        self.PORT=PORT
        self.Baudrate=Baudrate
        self.Climbcrc=ClimbRobotCRC()
        self.nodename=nodename
        self.writedatabuff=[]
        self.writepulsebuffer=[]
        self.writepulsespecialbuffer=[]
        self.readinfobuffer=[]
        self.ReadInfopublish=rospy.Publisher("/Registor_info_from_driver", String, queue_size=10)
        try:
            self.ser = serial.Serial(port=self.PORT, baudrate=self.Baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=1,timeout=0.3, xonxoff=0,rtscts=False,dsrdtr=False)
        except:
            print("Please check Usb port----")
    def Init_node(self):
        rospy.init_node(self.nodename)
    def Write_pulse_callback(self,msg):
        if len(self.writepulsebuffer)>10:
            self.writepulsebuffer=self.writepulsebuffer[1:]
            print(msg.data)
            self.writepulsebuffer.append(msg.data)
        else:
            print(msg.data)
            self.writepulsebuffer.append(msg.data)
    def Write_pulse_special_callback(self,msg):
        if len(self.writepulsespecialbuffer)>10:
            self.writepulsespecialbuffer=self.writepulsespecialbuffer[1:]
            print(msg.data)
            self.writepulsespecialbuffer.append(msg.data)
        else:
            print(msg.data)
            self.writepulsespecialbuffer.append(msg.data)
    def Read_info_callback(self,msg):
        if len(self.readinfobuffer)>10:
            self.readinfobuffer=self.readinfobuffer[1:]
            print(msg.data)
            self.readinfobuffer.append(msg.data)
        else:
            print(msg.data)
            self.readinfobuffer.append(msg.data)
    def Write_data_callback(self,msg):
        if len(self.writedatabuff)>10:
            self.writedatabuff=self.writedatabuff[1:]
            print(msg.data)
            self.writedatabuff.append(msg.data)
        else:
            print(msg.data)
            self.writedatabuff.append(msg.data)
    def Write_Read_topic_Sub(self,write_data_topicname,write_pulse_special_topicname,write_pulse_topicname,read_info_topicname):
        write_data_sub = rospy.Subscriber(write_data_topicname, String, self.Write_data_callback)
        write_pulse_special_sub = rospy.Subscriber(write_pulse_special_topicname, String, self.Write_pulse_special_callback)
        write_pulse_sub = rospy.Subscriber(write_pulse_topicname, String, self.Write_pulse_callback)
        read_info_sub = rospy.Subscriber(read_info_topicname, String, self.Read_info_callback)
        return write_data_sub,write_pulse_special_sub,write_pulse_sub,read_info_sub
    def get_serial_port(self):
        a,b=commands.getstatusoutput('python -m serial.tools.list_ports')
        print(b)
        return b
    def Send_PulseSpecial_to_driver(self,deviceaddr,function_code,PU24_3_1bit,PU16_2_3bit,PU8_15,PU0_7):
        """
        :param deviceaddr: just 0x78,you can use decimal or hexadecimal
        :param function_code:
        :param PU24_3_1bit:
        :param PU16_2_3bit:
        :param PU8_15:
        :param PU0_7:
        :return: [deviceaddr,function_code,PU8_15,PU0_7,PU24_3,PU16_2]
        """
        sendstring=self.Climbcrc.Combining_CRC_and_info([deviceaddr,function_code,PU24_3_1bit,PU16_2_3bit,PU8_15,PU0_7])
        readbuffer,strt=self.Send_message_to_port(sendstring)
        print("---port read buffer info---", readbuffer)
        return readbuffer
    def Send_Pulse_to_driver(self,deviceaddr,function_code,firstregistoraddr_high16,firstregistoraddr_low16,PU8_15,PU0_7,PU24_31,PU16_23,datalen=4,registornum_high16=0,registornum_low16=2):
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
        readbuffer,strt=self.Send_message_to_port(sendstring)
        print("---port read buffer info---", readbuffer)
        return readbuffer
    def Send_Data_to_driver(self,deviceaddr,function_code,firstregistoraddr_high16,firstregistoraddr_low16,data_high16,data_low16):
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
        readbuffer,strt=self.Send_message_to_port(sendstring)
        print("---port read buffer info---", readbuffer)
        return readbuffer
    def Read_info_from_driver(self,function_code,deviceaddr,registoraddr_high16,registoraddr_low16,firstregistoraddr_high16=0,firstregistoraddr_low16=0):
        """
        :param function_code: int here just 0x03 you can also use 3
        :param deviceaddr: int
        :param registoraddr_high16: int
        :param registoraddr_low16: int
        [0x01,0x03,0x00,0x00,0x00,0x01]
        :return:int list[function_code,deviceaddr,datalength,...,data_high16,data_low16...,CRCdata]
        """
        sendstring=self.Climbcrc.Combining_CRC_and_info([deviceaddr,function_code,firstregistoraddr_high16,firstregistoraddr_low16,registoraddr_high16,registoraddr_low16])
        # Modbus_Enable_info=[0x01,0x03,0x00,0x00,0x00,0x01]
        readbuffer,strt=self.Send_message_to_port(sendstring)
        print("---port read buffer info---", readbuffer)

        return readbuffer
    def Read_info_from_driver_string(self,cmd_string_withoutCRC):
        sendstring = self.Climbcrc.Combining_CRC_and_info(self.string_to_int_list(cmd_string_withoutCRC))
        readbuffer,strt = self.Send_message_to_port(sendstring)
        print("---port read buffer info---", readbuffer)
        self.ReadInfopublish.publish(strt[:len(strt)-2])
        print("Opreating return data,and cut the CRC data")
        return readbuffer
    def Write_info_driver_string(self,cmd_string_withoutCRC):
        sendstring = self.Climbcrc.Combining_CRC_and_info(self.string_to_int_list(cmd_string_withoutCRC))
        readbuffer,strt = self.Send_message_to_port(sendstring)
        print("---port read buffer info---", readbuffer)
        # self.ReadInfopublish.publish(strt[:len(strt)-2])
        # print "Opreating return data,and cut the CRC data"
        return readbuffer
    def Opreating_Info_CRC(self):
        pass
    def Position_abs_control(self):
        pass
    def Position_incremental_control(self):
        pass
    def Velocity_control(self):
        pass
    def string_to_int_list(self,stringinfo):
        return [int(int(i,16)) for i in wrap(stringinfo,2)]
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
        return readbuffer,strt
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
    write_data_topicname="climb_robot_write_data_topic"
    write_pulse_special_topicname="climb_robot_write_pulse_special_topic"
    write_pulse_topicname="climb_robot_write_pulse_topic"
    read_info_topicname="climb_robot_read_info_topic"
    climb_robot=ClimbRobot(PORT,baudrate,"Climbrobot_driver_node")
    climb_robot.get_serial_port()
    climb_robot.Init_node()
    climb_robot.Write_Read_topic_Sub(write_data_topicname,write_pulse_special_topicname,write_pulse_topicname,read_info_topicname)
    rate = rospy.Rate(30)

    # climb_robot.Disable_Modbus_serial()
    #
    # print climb_robot.Pulse_16bits_change(-100)
    count=0
    while not rospy.is_shutdown():
        if count==0:
            print("---Remember that you need to publish cmd string with out CRC check-----")
        elif count>10000:
            count=0
        if len(climb_robot.writedatabuff)!=0:
            writedata=climb_robot.writedatabuff[-1]
            print("receive from topic climb_robot_write_data_topic : ",writedata)
            climb_robot.Write_info_driver_string(writedata)
        else:
            pass
        if len(climb_robot.writepulsespecialbuffer)!=0:
            writedata=climb_robot.writepulsespecialbuffer[-1]
            print("receive from topic climb_robot_write_data_topic : ",writedata)
            climb_robot.Write_info_driver_string(writedata)
        else:
            pass
        if len(climb_robot.writepulsebuffer)!=0:
            writedata=climb_robot.writepulsebuffer[-1]
            print("receive from topic climb_robot_write_data_topic : ",writedata)
            climb_robot.Write_info_driver_string(writedata)
        else:
            pass
        if len(climb_robot.readinfobuffer)!=0:
            writedata=climb_robot.readinfobuffer[-1]
            print("receive from topic climb_robot_write_data_topic : ",writedata)
            climb_robot.Read_info_from_driver_string(writedata)
        else:
            pass
        count=+1
        rate.sleep()
if __name__ == "__main__":
    main()