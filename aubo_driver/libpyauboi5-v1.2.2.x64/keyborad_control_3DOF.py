#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:2----->roation
id:3------>Upper and lower climbing pole
"""
import serial
import time
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
import readchar
from robotcontrol import *
#import logging
# 创建一个logger
#logger = logging.getLogger()
class KeyControl3DOFROBOT():
    def __init__(self,PORT):
        self.PORT=PORT
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


    # def Control_3DOF_Robot(self,  control_id, velocity, outputPulse):  # position control
    #     """
    #
    #     :param master:
    #     :param control_id: 1-stand,2-rotation,3-climber
    #     :param velocity: 0-2500
    #     :param outputPulse: High 32位
    #     :return:
    #     """
    #     logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 0, 8))
    #     # print type(master.execute(4, cst.READ_HOLDING_REGISTERS, 0, 8))
    #     logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 1, output_value=6))  # enable Climb Driver
    #     logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=1))  # enable Climb Driver
    #     logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 290,
    #                                output_value=-5))  # High 16 10000 pulse 1 rpm,negtive up,positive up
    #     #logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 291, output_value=outputPulse))  # Low 16bit
    #     logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 97, output_value=velocity))  # internal velocity
    #     # logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 113, output_value=1000))  # internal velocity
    #     # logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 114, output_value=1000))  # internal velocity
    #     logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 324, output_value=1000))  # set fixed velocity
    #     #
    #     logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 1))
    #     logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 214, 1))
    #     logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 218, 1))
    #     logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 220, 1))
    #
    #     logger.info(master.execute(control_id, cst.READ_HOLDING_REGISTERS, 212, 12))
    def Control_3DOF_Robot(self,  velocity, outputPulse,control_id):
        logger = modbus_tk.utils.create_logger("console")

        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=self.PORT, baudrate=19200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            logger.info("connected")
            logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 1, output_value=6))  # enable Climb Driver
            # logger.info(
            #     master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=1))  # enable Climb Driver
            logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 290,
                                       output_value=outputPulse * 24.0 / 136.0))  # High 16 10000 pulse 1 rpm,negtive up,positive up
            # logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 291, output_value=outputPulse))  # Low 16bit
            logger.info(
                master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 97, output_value=velocity))  # internal velocity
            # return master
        except modbus_tk.modbus.ModbusError as exc:
            logger.error("%s- Code=%d", exc, exc.get_exception_code())

    def Holding_Robot(self, velocity, outputDistance, control_id=1):  # position control
        """

        :param master:
        :param velocity:
        :param outputPulse:5.5cm -20 Negtive up,Positive Down
        :param control_id:
        :return:
        """
        outputPulse = outputDistance * 20 / 5.5
        self.Control_3DOF_Robot( velocity, outputPulse,control_id)


    def Rotation_Robot(self,velocity, outputDegree, control_id=2):  # position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDegree: 0-360Degree,Positive clockwise,Negtive disclockwise
        :param control_id:
        :return:
        """
        logger.info("outputDegree: 0-360 Degree,Positive clockwise,Negtive disclockwise")
        outputPulse = outputDegree / 6.5
        self.Control_3DOF_Robot( velocity, outputPulse,control_id)


    def Climbing_Robot(self, velocity, outputDistance, control_id):  # position control
        """

        :param master:
        :param velocity: 0-2500
        :param outputDistance: 0-300cm
        :param control_id:
        :return:
        """

        logger.info("outputDegree: 0-360 Degree,Positive down,Negtive up")
        outputPulse = outputDistance * 24.0 / 136.0
        self.Control_3DOF_Robot( velocity, outputPulse,control_id)


    def Read_3DOF_Controller_Buffe(self):
        """

        :param master:
        :return:
        """
        logger = modbus_tk.utils.create_logger("console")

        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=self.PORT, baudrate=19200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            logger.info("connected")


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
        except modbus_tk.modbus.ModbusError as exc:
            logger.error("%s- Code=%d", exc, exc.get_exception_code())


    def Emergency_Stop_All(self, control_id, All_Stop_flag):
        logger = modbus_tk.utils.create_logger("console")

        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=self.PORT, baudrate=19200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            logger.info("connected")
            if All_Stop_flag == 1:
                logger.info(master.execute(1, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))
                logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))
                logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))
            else:
                logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=0))  # enable Climb Driver

        except modbus_tk.modbus.ModbusError as exc:
            logger.error("%s- Code=%d", exc, exc.get_exception_code())
    def Open_Stop_Enable(self,  control_id,stop_open_flag):
        logger = modbus_tk.utils.create_logger("console")

        try:
            # Connect to the slave
            master = modbus_rtu.RtuMaster(
                serial.Serial(port=self.PORT, baudrate=19200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
            )
            master.set_timeout(5.0)
            master.set_verbose(True)
            # logger.info("connected")
            logger.info(master.execute(control_id, cst.WRITE_SINGLE_REGISTER, 282, output_value=stop_open_flag))  # enable
            return control_id,stop_open_flag
        except modbus_tk.modbus.ModbusError as exc:
            logger.error("%s- Code=%d", exc, exc.get_exception_code())
def main():
    flag=1
    PORT="/dev/ttyUSB0"
    W_count=0
    KeyCheck=KeyControl3DOFROBOT(PORT)
    Temp_control_id_flag=0
    Temp_open_stop_flag=0
    try:
        # Master=KeyCheck.Connect_3DOF_MODbus_RTU()
        print "Press 'H' to Enable Hold"
        print "      'R' to Enable Rotation"
        print "      'C' to Enable Climbing"
        print "      'Y' to Close Hold"
        print "      'U' to Close Rotation"
        print "      'E' to Close Climbing"
        print "      'W' to climb or Hold UP"
        print "      'X' to climb or Hold Down"
        print "      'A' to Rotation clockwise"
        print "      'D' to Rotation disclockwise"
        print "      'R' to Read the information About the driver"
        print "      '?' to Help Infor"
        print "      'Enter' to quit the program..."
        while True:

            # Read a key

            key = readchar.readkey()
            if(key == 'H'):
                print "-------Enable Hold-------"
                Temp_control_id_flag,Temp_open_stop_flag=KeyCheck.Open_Stop_Enable(1,1)
            elif(key == 'R'):
                print "-----Enable Rotation----"
                Temp_control_id_flag,Temp_open_stop_flag=KeyCheck.Open_Stop_Enable( 2,1)
            elif(key == 'C'):
                print "------Enable Climbing------"
                Temp_control_id_flag,Temp_open_stop_flag=KeyCheck.Open_Stop_Enable( 3,1)
            elif(key == 'Y'):
                print "------Close Hold------"
                Temp_control_id_flag,Temp_open_stop_flag=KeyCheck.Open_Stop_Enable( 1, 0)
            elif(key == 'U'):
                print "------Close Rotation-----"
                Temp_control_id_flag,Temp_open_stop_flag=KeyCheck.Open_Stop_Enable( 2, 0)
            elif(key == 'E'):
                print "Close Climbing"
                Temp_control_id_flag,Temp_open_stop_flag=KeyCheck.Open_Stop_Enable( 3, 0)
            elif(key == 'W'):
                print "-------climb or Hold UP-----"
                if Temp_control_id_flag==1 and Temp_open_stop_flag==1:
                    print "Control Hold"
                    if W_count>-40:
                        W_count-=1
                        # print W_count
                        KeyCheck.Holding_Robot(1000,W_count,1)
                    else:
                        pass
                elif Temp_control_id_flag == 3 and Temp_open_stop_flag == 1:
                    print "Control climb"
                    if W_count>-40:
                        W_count-=1
                        KeyCheck.Control_3DOF_Robot(1000,W_count,3)
                        time.sleep(0.5)

                    else:
                        pass
                else:
                    pass

                print "W_count",W_count
            elif(key == 'X'):
                print "------climb or Hold Down-----"
                if Temp_control_id_flag == 1 and Temp_open_stop_flag == 1:
                    print "Control Hold"
                    if W_count < 40:
                        W_count += 1
                        # print W_count
                        KeyCheck.Holding_Robot( 1000, W_count)
                    else:
                        pass
                elif Temp_control_id_flag == 3 and Temp_open_stop_flag == 1:
                    print "Control climb"
                    if W_count < 40:
                        W_count += 1
                        KeyCheck.Control_3DOF_Robot(1000,W_count,3)
                    else:
                        pass
                else:
                    pass

                print "W_count", W_count
            elif(key == 'A'):
                print "-----Rotation clockwise------"
                if Temp_control_id_flag == 2 and Temp_open_stop_flag == 1:
                    print "Control Hold"
                    if W_count < 360:
                        W_count += 5
                        # print W_count
                        KeyCheck.Rotation_Robot( 1000, W_count)
                    else:
                        pass
                else:
                    pass

                print "W_count", W_count
            elif(key == 'D'):
                print "-------Rotation disclockwise------"
                if Temp_control_id_flag == 2 and Temp_open_stop_flag == 1:
                    print "Control Hold"
                    if W_count >- 360:
                        W_count -= 5
                        # print W_count
                        KeyCheck.Rotation_Robot( 1000, W_count)
                    else:
                        pass
                else:
                    pass
            elif(key == 'R'):
                print "-------Read Information------"
                KeyCheck.Read_3DOF_Controller_Buffe()
            elif(key == '?'):
                print "Press 'H' to Enable Hold"
                print "      'R' to Enable Rotation"
                print "      'C' to Enable Climbing"
                print "      'Y' to Close Hold"
                print "      'U' to Close Rotation"
                print "      'E' to Close Climbing"
                print "      'W' to climb or Hold UP"
                print "      'X' to climb or Hold Down"
                print "      'A' to Rotation clockwise"
                print "      'D' to Rotation disclockwise"
                print "      'R' to Read the information About the driver"
                print "      '?' to Help Infor"
                print "      'Enter' to quit the program..."
            elif(key == '\r'):
                print "Exiting..."
                break
            else:
                print "Please Use only allowed keys:  Enter!"
    except:
        print "Please check the serial port------"
if __name__=="__main__":
    main()