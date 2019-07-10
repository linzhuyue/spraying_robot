#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from std_msgs.msg import String,Float64,Bool
from robotcontrol import *
import serial
import time
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

class AuboTrajectory():
    def __init__(self,Aubo_IP,AuboWorkSpaceLen,AuboWorkSpaceWidth,Port):
        self.AuboWorkSpaceLen=AuboWorkSpaceLen
        self.AuboWorkSpaceWidth=AuboWorkSpaceWidth
        self.Aubo_IP=Aubo_IP
        self.OpenState = rospy.Subscriber("/open_aubo_state_flag", Bool, self.Open_Aubo_callback)
        self.PubState = rospy.Publisher("/open_aubo_state_flag", Bool, queue_size=10)
        self.Port=Port
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
        logger_init()
        # 启动测试
        logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))
        # 系统初始化
        Auboi5Robot.initialize()
        # 创建机械臂控制类
        robot = Auboi5Robot()
        # 创建上下文
        handle = robot.create_context()
        # 打印上下文
        logger.info("robot.rshd={0}".format(handle))
        try:

            # 链接服务器
            ip = self.Aubo_IP#'192.168.1.11'
            port = 8899
            result = robot.connect(ip, port)
            if result != RobotErrorType.RobotError_SUCC:
                logger.info("connect server{0}:{1} failed.".format(ip, port))
            else:
                # # 重新上电
                # robot.robot_shutdown()
                #
                # # 上电
                # robot.robot_startup()
                #
                # # 设置碰撞等级
                # robot.set_collision_class(7)

                # 设置工具端电源为１２ｖ
                # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

                # 设置工具端ＩＯ_0为输出
                robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

                # 获取工具端ＩＯ_0当前状态
                tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
                logger.info("tool_io_0={0}".format(tool_io_status))

                # 设置工具端ＩＯ_0状态
                robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

                # 获取控制柜用户DI
                io_config = robot.get_board_io_config(RobotIOType.User_DI)

                # 输出DI配置
                logger.info(io_config)

                # 获取控制柜用户DO
                io_config = robot.get_board_io_config(RobotIOType.User_DO)

                # 输出DO配置
                logger.info(io_config)
                # 当前机械臂是否运行在联机模式
                logger.info("robot online mode is {0}".format(robot.is_online_mode()))
        except RobotError, e:
            logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))
        return robot
    def Connect_3DOF_MODbus_RTU(self):
        logger = modbus_tk.utils.create_logger("console")

        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=self.PORT, baudrate=19200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            logger.info("connected")
            return master
        except modbus_tk.modbus.ModbusError as exc:
            logger.error("%s- Code=%d", exc, exc.get_exception_code())
    def Control_3DOF_Robot(self,master,control_id,velocity,outputPulse):#position control
        """

        :param master:
        :param control_id: 1-stand,2-rotation,3-climber
        :param velocity: 0-2500
        :param outputPulse: High 32位
        :return:
        """
        logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 0, 8))
        # print type(master.execute(4, cst.READ_HOLDING_REGISTERS, 0, 8))
        logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 1, output_value=6))#enable Climb Driver
        logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=1))  # enable Climb Driver
        logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 290, output_value=outputPulse)) # High 16 10000 pulse 1 rpm,negtive up,positive up
        logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 291, output_value=outputPulse))#Low 16bit
        logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 97, output_value=velocity))  # internal velocity
        # logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 113, output_value=1000))  # internal velocity
        # logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 114, output_value=1000))  # internal velocity
        logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 324, output_value=1000))  # set fixed velocity
        #
        logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 1))
        logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 214, 1))
        logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 218, 1))
        logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 220, 1))

        logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 12))
    def Holding_Robot(self,master,velocity,outputDistance,control_id=1):#position control
        """

        :param master:
        :param velocity:
        :param outputPulse:5.5cm -20 Negtive up,Positive Down
        :param control_id:
        :return:
        """
        outputPulse=outputDistance/3.6
        self.Control_3DOF_Robot(master,control_id,velocity,outputPulse)
    def Rotation_Robot(self,master,velocity,outputDegree,control_id=2):#position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDegree: 0-360Degree,Positive clockwise,Negtive disclockwise
        :param control_id:
        :return:
        """
        logger.info("outputDegree: 0-360 Degree,Positive clockwise,Negtive disclockwise")
        outputPulse=outputDegree/6.5
        self.Control_3DOF_Robot(master, control_id, velocity, outputPulse)

    def Climbing_Robot(self,master,velocity,outputDistance,control_id=3):#position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDistance: 0-300cm
        :param control_id:
        :return:
        """
        logger.info("outputDegree: 0-360 Degree,Positive down,Negtive up")
        outputPulse=outputDistance /5.6
        self.Control_3DOF_Robot(master, control_id, velocity, outputPulse)
    def Read_3DOF_Controller_Buffe(self,master):
        """

        :param master:
        :return:
        """
        logger.info("Driver Warnning nums Meaning Table:")
        logger.info("0: No Warnning")
        logger.info("3: Over Flow")
        logger.info("4: Over heat")
        logger.info("6: Encoder Warnning")
        logger.info("13: EEPROM WRITING&READING Unusal")
        logger.info("8: Over Load")
        logger.info("11: Over speed")
        logger.info("2: Over Voltage")
        logger.info("1: Lack Voltage")
        logger.info("9: Position Error Large")
        logger.info(master.execute(1, cst.READ_HOLDING_REGISTERS, 212, 2))
        logger.info("Holding Robot driver warnning nums")
        logger.info(master.execute(1, cst.READ_HOLDING_REGISTERS, 202, 2))
        logger.info("Rotation Robot command position counts")
        logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 212, 2))
        logger.info("Rotation Robot driver warnning nums")
        logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 202, 2))
        logger.info("Climbing Robot command position counts")
        logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 212, 2))
        logger.info("Climbing Robot driver warnning nums")
        logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 202, 2))
    def Emergency_Stop_All(self,master,control_id,All_Stop_flag):
        if All_Stop_flag==1:
            logger.info(master.execute(1, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))
            logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))
            logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))
        else:
            logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))  # enable Climb Driver

    def DisConnect_Aubo(self,auboRobot):
        # 断开服务器链接
        if auboRobot.connected:
            # 关闭机械臂
            auboRobot.robot_shutdown()
            # 断开机械臂链接
            auboRobot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))
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
        logger.info("joint_status={0}".format(joint_status))
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
        logger.info("fk--------")
        logger.info(fk_ret)
        return fk_ret
    def Aubo_inverse_kinematics(self,robot,jointangular,newpose,neworientaion_Quaternion):
        # 获取关节最大加速度
        logger.info(robot.get_joint_maxacc())
        joint_radian = jointangular#self.deg_to_rad(jointangular)
        pos_test = newpose#(-0.5672477590258516, 0.51507448660946279, 0.57271770314023)  # the right
        ori_test = neworientaion_Quaternion#(0.49380082661500474, 0.5110471735827042, -0.5086787259664434, -0.48604267688817565)
        logger.info("----ik----after--------")
        ik_result = robot.inverse_kin(joint_radian, pos_test, ori_test)
        logger.info(ik_result)
        return ik_result
    def Aubo_Line_trajectory(self,robot,start_point,End_point,):
        joint_radian = self.deg_to_rad(start_point)
        logger.info("move joint to {0}".format(joint_radian))
        robot.move_joint(joint_radian)
        joint_radian = self.deg_to_rad(End_point)
        logger.info("move joint to {0}".format(joint_radian))
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
        logger.info("move joint to {0}".format(joint_radian))
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
        logger.warning("Right now,Our Robot ARM can just run in a 1.1X0.7 square meter")
        logger.warning("So you can't set up Sector_Length and Sector_Width more than 1.1 and 0.7,respectively")
        logger.warning("This controller set arm in planning central by default")
        Sector_Nums_Calcu = self.AuboWorkSpaceWidth/Sector_Width
        if Sector_Nums_Calcu<Sector_Nums and Sector_Length >self.AuboWorkSpaceLen and Sector_Width>self.AuboWorkSpaceWidth and Sector_Length <0 and Sector_Width <0:
            logger.info("Please check your Sector length , Sector width and Sector_Nums")
        else:
            logger.info("Go to the start point")
            self.Aubo_Move_to_Point(robot,StartPoint)
            Temp_joint_angular =self.deg_to_rad(StartPoint)

            logger.info("Go to the left first")
            forward_kinemtics=self.Aubo_forward_kinematics(robot,StartPoint)
            pose=forward_kinemtics['pos']
            orient=forward_kinemtics['ori']
            Res_Left, Res_Right,Last_Queue=self.Caculate_Path_point(Sector_Nums,Sector_Width,Sector_Length,pose,Left_Right_Flag)
            for i in range(len(Last_Queue)):
                logger.info("go to the {0} point".format(i))
                ik_result=self.Aubo_inverse_kinematics(robot,Temp_joint_angular,Last_Queue[i],orient)
                joint_radian = tuple(ik_result['joint'])

                logger.info("move joint to {0}".format(joint_radian))
                robot.move_line(joint_radian)
                Temp_joint_angular=joint_radian
            logger.info("Path planning OK,Go back to start point----")
            self.Aubo_Move_to_Point(robot, StartPoint)
            self.DisConnect_Aubo(robot)

def main():
    ratet=1
    IP='192.168.1.11'
    Port='/dev/ttyUSB0'
    StartPoint=(10.3,3.6,-89.9,-91.2,-86.04,0.15)
    Sector_Length=1.0 #m
    Sector_Width=0.2 #m
    Sector_Nums=3
    Left_Right_Flag =1
    Aub=AuboTrajectory(IP,1.1,0.7,Port)
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    flag_roation=0
    count=0
    try:

        Robot=Aub.Init_aubo_driver()
        Master = Aub.Connect_3DOF_MODbus_RTU()
        while not rospy.is_shutdown():
            if len(Aub.OpenstateBool)!=0:
                if Aub.OpenstateBool[-1]:
                    time.sleep(2)
                    logger.info("Sleep time is over,then Climb starts opreating task")
                    Aub.Climbing_Robot(Master,2000,-136)#136cm,-136up
                    time.sleep(5)
                    logger.info("Sleep time is over,then aubo starts opreating task")
                    Aub.Spray_Painting_Cartesian_Sector_Planning(Robot,StartPoint,Sector_Length,Sector_Width,Sector_Nums,Left_Right_Flag)
                    #Aub.DisConnect_Aubo(Robot)
                    time.sleep(8)
                    logger.info("Sleep time is over,then climb robot goes to initial point")
                    Aub.Climbing_Robot(Master,2000,0)#136cm,-136up
                    time.sleep(5)
                    Aub.Spray_Painting_Cartesian_Sector_Planning(Robot,StartPoint,Sector_Length,Sector_Width,Sector_Nums,Left_Right_Flag)
                    #Aub.DisConnect_Aubo(Robot)
                    time.sleep(8)

                    if flag_roation==0:
                        logger.info("Sleep time is over,then Rotation robot goes to -90 degree in disclockwise")
                        Aub.Rotation_Robot(Master,1000,-90)
                        time.sleep(5)
                        flag_roation=1
                    logger.info("Sleep time is over,then Ros publishs close flag topic")
                    Aub.Read_3DOF_Controller_Buffe(Master)
                    Aub.PubState.publish(False)
                    count+=1
                    if count>=3:
                        Aub.DisConnect_Aubo(Robot)
                else:
                    logger.warning("Please wait Mobile platform waypoint over")
            rate.sleep()
    except:
        logger.error("Aubo or Climb robot disconnect,Please check those devices.!")
    # finally:
    #     Aub.DisConnect_Aubo(Robot)
if __name__ == '__main__':
    main()