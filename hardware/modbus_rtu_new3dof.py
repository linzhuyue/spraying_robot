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
class ClimbRobot:
    def __init__(self,PORT,Baudrate):
        self.PORT=PORT
        self.Baudrate=Baudrate
        self.logger = modbus_tk.utils.create_logger("console")
        self.master = modbus_rtu.RtuMaster(
            serial.Serial(port=self.PORT, baudrate=self.Baudrate, bytesize=8, parity='O', stopbits=1, xonxoff=0)
        )
    def Init_node(self):
        pass
    def Init_Modbus(self):
        try:
            self.master.set_timeout(5.0)
            self.master.set_verbose(True)
            self.logger.info("connected")
        except modbus_tk.modbus.ModbusError as exc:
            self.logger.error("%s- Code=%d", exc, exc.get_exception_code())
    def Test_read_Modbus(self):
        self.logger.info(self.master.execute(1, cst.READ_HOLDING_REGISTERS, 0, 8))
    def Enable_Modbus(self):
        self.logger.info(self.master.execute(1, cst.WRITE_SINGLE_REGISTER, 0, output_value=1))
        # self.logger.info(self.master.execute(1, cst.WRITE_SINGLE_REGISTER, 1, output_value=1))
#PORT = '/dev/ttyp5'
def main():
    PORT = "/dev/ttyUSB0"
    baudrate=19200
    climb_robot=ClimbRobot(PORT,baudrate)
    climb_robot.Init_Modbus()
    climb_robot.Enable_Modbus()
    # climb_robot.Test_read_Modbus()


        # logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 8))
        # logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 0, 8))
        # #change to SigIn SON enable driver
        #
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 3, output_value=1)


if __name__ == "__main__":
    main()