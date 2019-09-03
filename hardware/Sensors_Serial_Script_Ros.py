#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# import sys
# import serial
# import commands
# import time
# from CRC_16_Check import *
# from textwrap import wrap
# from std_msgs.msg import String,Int64
# import yaml

class SensorsRobot:
    def __init__(self,PORT=None,Baudrate=115200,nodename="haha"):
        # self.readstringlength=25

        # self.Climbcrc=RobotCRC16()
        # self.path_current=os.getcwd()
        # self.readinfobuffer=[]
        # self.yamlDic={}
        self.PORT=PORT#self.yamlDic['port']
        self.Baudrate=Baudrate#self.yamlDic['baudrate']
        self.nodename=nodename#self.yamlDic['nodename']
        # self.ReadInfopublish=rospy.Publisher("/Registor_info_from_sensors", String, queue_size=10)
        
        # try:
        #     print "haha"
        #     #self.ser = serial.Serial(port=self.PORT, baudrate=self.Baudrate, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=1,timeout=0.3, xonxoff=0,rtscts=True,dsrdtr=True)
        # except:
        #     print("Please check Usb port----")
    def Set_PORT(self):
        self.PORT=self.yamlDic['port']
    def Set_Baudrate(self):
        self.Baudrate=self.yamlDic['baudrate']
    def Set_nodename(self):
        self.nodename=self.yamlDic['nodename']
    def Opreating_Yaml(self):

        yaml_path = str(self.path_current)+"/src/mobile_sensors/config/"+self.configname
        # print yaml_path
        file_data = open(yaml_path)
        self.yamlDic = yaml.load(file_data)
        file_data.close()
    def Init_node(self):
        rospy.init_node(self.nodename)
        # self.Opreating_Yaml()
    def get_serial_port(self):
        a,b=commands.getstatusoutput('python -m serial.tools.list_ports')
        print(b)
        return b

    def Send_message_to_port(self,message):
        """
        :param message: String "010600000001480A" or "01 06 00 00 00 01 48 0A"
        :return: int list
        """
        message_bytes = message.replace(" ",'').decode('hex')
        print message_bytes
        # print str(hex(message_bytes))
        self.ser.write(message_bytes)
        self.ser.flushInput()
        self.ser.flushOutput()
        time.sleep(0.01)
        strt = self.ser.read(self.readstringlength).encode('hex')
        readbuffer=[int(int(i, 16)) for i in wrap(strt, 2)]
        return readbuffer,strt
    def Send_message_to_port_CRC(self,command_num):
        """
        command_num:读取yaml文件的命令列表
        """
        senstr=self.Climbcrc.Combining_CRC_and_info(command_num)
        tempbuffer=climb_robot.Send_message_to_port(senstr)
        return tempbuffer
    def Read_information_from_yaml(self,cmdstr):
        return self.yamlDic['cmdstr']
def main():
    # Baudrate=
    # PORT=yamlDic['port']
    # nodename=
    climb_robot=ClimbRobot()
    # climb_robot.get_serial_port()
    climb_robot.Init_node()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        print 'hah'
        # print climb_robot.Send_message_to_port_CRC(climb_robot.Read_information_from_yaml('readD11Registor'))
        rate.sleep()
if __name__ == "__main__":
    main()