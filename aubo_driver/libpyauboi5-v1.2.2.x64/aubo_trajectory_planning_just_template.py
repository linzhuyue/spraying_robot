#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool
from robotcontrol import *
import serial
import time
import scipy.io as sio

import os
class AuboTrajectory():
    def __init__(self,Aubo_IP,AuboWorkSpaceLen,AuboWorkSpaceWidth):
        self.AuboWorkSpaceLen=AuboWorkSpaceLen
        self.AuboWorkSpaceWidth=AuboWorkSpaceWidth
        self.Aubo_IP=Aubo_IP
        self.OpenState = rospy.Subscriber("/open_aubo_state_flag", Bool, self.Open_Aubo_callback)
        self.PubState = rospy.Publisher("/open_aubo_state_flag", Bool, queue_size=10)
        self.OpenstateBool=[False]
    def Init_node(self):
        rospy.init_node("move_aubo_trajectory_planning")

    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * math.pi / 180)
        return tuple(dd)
    def Init_aubo_driver(self):
        # 初始化logger
        #logger_init()
        # 启动测试
        print("{0} test beginning...".format(Auboi5Robot.get_local_time()))
        # 系统初始化
        Auboi5Robot.initialize()
        # 创建机械臂控制类
        robot = Auboi5Robot()
        # 创建上下文
        handle = robot.create_context()
        # 打印上下文
        print("robot.rshd={0}".format(handle))
        try:

            # 链接服务器
            ip = self.Aubo_IP#'192.168.1.11'
            port = 8899
            result = robot.connect(ip, port)
            if result != RobotErrorType.RobotError_SUCC:
                print("connect server{0}:{1} failed.".format(ip, port))
            else:
                # # 重新上电
                # robot.robot_shutdown()
                #
                # # 上电
                #robot.robot_startup()
                #
                # # 设置碰撞等级
                # robot.set_collision_class(7)

                # 设置工具端电源为１２ｖ
                # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

                # 设置工具端ＩＯ_0为输出
                robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

                # 获取工具端ＩＯ_0当前状态
                tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
                print("tool_io_0={0}".format(tool_io_status))

                # 设置工具端ＩＯ_0状态
                robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

                # 获取控制柜用户DI
                io_config = robot.get_board_io_config(RobotIOType.User_DI)

                # 输出DI配置
                print(io_config)

                # 获取控制柜用户DO
                io_config = robot.get_board_io_config(RobotIOType.User_DO)

                # 输出DO配置
                print(io_config)
                # 当前机械臂是否运行在联机模式
                print("robot online mode is {0}".format(robot.is_online_mode()))
        except RobotError, e:
            logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))
        return robot

    def DisConnect_Aubo_No_ShutDown(self,auboRobot):
        # 断开服务器链接
        auboRobot.disconnect()
    def DisConnect_Aubo(self,auboRobot):
        # 断开服务器链接
        if auboRobot.connected:
            # 关闭机械臂
            auboRobot.robot_shutdown()
            # 断开机械臂链接
            auboRobot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        print("{0} test completed.".format(Auboi5Robot.get_local_time()))
    def Open_Aubo_callback(self,msg):
        if len(self.OpenstateBool)>10:
            self.OpenstateBool=self.OpenstateBool[1:]
            self.OpenstateBool.append(msg.data)
        else:
            self.OpenstateBool.append(msg.data)
        # print msg
        self.Openstate=msg.data
    def Aubo_trajectory_init(self,robot,maxacctuple,maxvelctuple):
        joint_status = robot.get_joint_status()
        print("joint_status={0}".format(joint_status))
        # 初始化全局配置文件
        robot.init_profile()
        # 设置关节最大加速度
        robot.set_joint_maxacc(maxacctuple)#(2.5, 2.5, 2.5, 2.5, 2.5, 2.5)

        # 设置关节最大加速度
        robot.set_joint_maxvelc(maxvelctuple)#(1.5, 1.5, 1.5, 1.5, 1.5, 1.5)
        # 设置机械臂末端最大线加速度(m/s)
        robot.set_end_max_line_acc(0.5)
        # 获取机械臂末端最大线加速度(m/s)
        # robot.set_end_max_line_velc(0.2)
        robot.set_end_max_line_velc(0.5)
    def Aubo_forward_kinematics(self,robot,jointangular):
        joint_radian = self.deg_to_rad(jointangular)
        fk_ret = robot.forward_kin(joint_radian)
        print("fk--------")
        print(fk_ret)
        return fk_ret
    def Aubo_inverse_kinematics(self,robot,jointangular,newpose,neworientaion_Quaternion):
        # 获取关节最大加速度
        print(robot.get_joint_maxacc())
        joint_radian = jointangular#self.deg_to_rad(jointangular)
        print("pose and ori")
        print(newpose)
        print(neworientaion_Quaternion)
        pos_test = newpose#(-0.5672477590258516, 0.51507448660946279, 0.57271770314023)  # the right
        ori_test = neworientaion_Quaternion#(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
        print("----ik----after--------")
        ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
        print(ik_result)
        return ik_result
    def Aubo_Line_trajectory(self,robot,start_point,End_point,):
        joint_radian = self.deg_to_rad(start_point)
        print("move joint to {0}".format(joint_radian))
        robot.move_joint(joint_radian)
        joint_radian = self.deg_to_rad(End_point)
        print("move joint to {0}".format(joint_radian))
        robot.move_line(joint_radian)
    def Caculate_Path_point(self,Sector_Nums,Sector_Width,Sector_Length,pose,Left_Right_Flag):
        Res_Left=[]
        Res_Right=[]
        Last_Queue=[]
        for i in range(Sector_Nums):
            Res_Left.append({i:(pose[0],pose[1]-Sector_Length/2.0,pose[2]-i*Sector_Width)})
            Res_Right.append({i:(pose[0], pose[1] + Sector_Length / 2.0, pose[2]-i*Sector_Width)})
        if Left_Right_Flag:
            for i in range(len(Res_Left)):
                if i%2==0:#Even
                    Last_Queue.append(Res_Left[i][i])
                    Last_Queue.append(Res_Right[i][i])
                else:
                    Last_Queue.append(Res_Right[i][i])
                    Last_Queue.append(Res_Left[i][i])
        else:
            if i % 2 != 0:  # Even
                Last_Queue.append(Res_Left[i][i])
                Last_Queue.append(Res_Right[i][i])
            else:
                Last_Queue.append(Res_Right[i][i])
                Last_Queue.append(Res_Left[i][i])
        return Res_Left,Res_Right,Last_Queue
    def Aubo_Move_to_Point(self,robot,jointAngular):
        joint_radian = self.deg_to_rad(jointAngular)
        print("move joint to {0}".format(joint_radian))
        robot.move_joint(joint_radian)
    def Spray_Painting_Cartesian_Sector_Planning(self,robot,StartPoint,Sector_Length,Sector_Width,Sector_Nums,Left_Right_Flag):
        """

        :param robot:
        :param StartPoint: joint angular tuple
        :param Sector_Length: sector length
        :param Sector_Width: sector width
        :param Sector_Nums: sector Nums
        :param Left_Right_Flag:
        :return:
        """
        print("Right now,Our Robot ARM can just run in a 1.1X0.7 square meter")
        print("So you can't set up Sector_Length and Sector_Width more than 1.1 and 0.7,respectively")
        print("This controller set arm in planning central by default")
        Sector_Nums_Calcu = self.AuboWorkSpaceWidth/Sector_Width
        if Sector_Nums_Calcu<Sector_Nums and Sector_Length >self.AuboWorkSpaceLen and Sector_Width>self.AuboWorkSpaceWidth and Sector_Length <0 and Sector_Width <0:
            print("Please check your Sector length , Sector width and Sector_Nums")
        else:
            print("Go to the start point")
            self.Aubo_Move_to_Point(robot,StartPoint)
            Temp_joint_angular =self.deg_to_rad(StartPoint)

            print("Go to the left first")
            forward_kinemtics=self.Aubo_forward_kinematics(robot,StartPoint)
            pose=tuple(forward_kinemtics['pos'])
            orient=tuple(forward_kinemtics['ori'])
            Res_Left, Res_Right,Last_Queue=self.Caculate_Path_point(Sector_Nums,Sector_Width,Sector_Length,pose,Left_Right_Flag)
            print(Last_Queue)
            for i in range(len(Last_Queue)):
                print("go to the {0} point".format(i))
                print(Temp_joint_angular)
                ik_result=self.Aubo_inverse_kinematics(robot,Temp_joint_angular,Last_Queue[i],orient)
                print(orient)
                joint_radian = tuple(ik_result['joint'])

                print("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)
                Temp_joint_angular=joint_radian
            print("Path planning OK,Go back to start point----")
            self.Aubo_Move_to_Point(robot, StartPoint)
            # 清除所有已经设置的全局路点
            robot.remove_all_waypoint()
            #self.DisConnect_Aubo_No_ShutDown(robot)

def main():
    ratet=1
    IP='192.168.1.11'
    StartPoint=(-3.3364,12.406,-81.09,-91.207,-86.08,0.164)
    Sector_Length=0.8 #m
    Sector_Width=0.2 #m
    Sector_Nums=4
    Left_Right_Flag =1
    maxacctuple=(2.5, 2.5, 2.5, 2.5, 2.5, 2.5)
    maxvelctuple=(1.5, 1.5, 1.5, 1.5, 1.5, 1.5)
    Aub=AuboTrajectory(IP,1.1,0.7,Port)
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    flag_roation=0
    count=0
    #try:
    try:
        # Robot=Aub.Init_aubo_driver()
        # Aub.Aubo_trajectory_init(Robot,maxacctuple,maxvelctuple)
        Robot = Aub.Init_aubo_driver()
        Aub.Aubo_trajectory_init(Robot, maxacctuple, maxvelctuple)
    except:
        print "init 3DOF not OK"
    try:
        while not rospy.is_shutdown():
            pass
            rate.sleep()
    except:
        pass
    #except:
      # logger.error("Aubo or Climb robot disconnect,Please check those devices.!")
    # finally:
    #     Aub.DisConnect_Aubo(Robot)
if __name__ == '__main__':
    main()