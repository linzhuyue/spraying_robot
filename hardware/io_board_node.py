#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Ioboard class.

python serial port code for 24 io board
the code '55C81900000055' means closing all io port
and the code '55C819FFFFFF55' means opening all io port
You can also control one relay:
first relay open:55C8010155
first relay close:55C8010055
second relay open:55C8020155
second relay close:55C8020055
.
.
.
.
24th relay open:55C8 18 01 55
24th relay close:55C8 18 00 55
@author: Lzyue
KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="SerialIo0"
For 8 ioboard
without delaying:
55C8020155 open #port 2
55c8020055 close
"""
import rospy
import time
from std_msgs.msg import String
import serial
from time import sleep
class Io_board():
    def __init__(self,nodename):
        self.nodename=nodename
        self.iostatebuff=[]
    def Init_node(self):
        rospy.init_node(self.nodename)
    def Io_callback(self,msg):
        print msg.data
        self.iostatebuff.append(msg.data)
    def Io_Sub(self,topicname):
        sub = rospy.Subscriber(topicname, String, self.Io_callback)
    def recv(self,serial):
        while True:
            data = serial.read_all()
            if data == '':
                continue
            else:
                break
            time.sleep(0.02)
        return data
def main():
    iob=Io_board("Io_board_node")
    iob.Init_node()
    iob.Io_Sub("io_state")
    import serial
    serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)  #/dev/ttyUSB0
    if serial.isOpen() :
        print("open port success")
    else :
        print("open port failed")
    rate = rospy.Rate(4)
    #rostopic pub /io_state std_msgs/String "55C8070155"
    while not rospy.is_shutdown():
        #data =recv(serial)
        if len(iob.iostatebuff)!=0:
            iocmd=iob.iostatebuff[-1]
            print("receive : ",iocmd)
            time.sleep(1)
            serial.write(iocmd.decode("hex")) #数据写回
        else:
            pass
        # data ='55C81900000055'
        rate.sleep()
if __name__ == '__main__':
    main()