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

PORT = "/dev/ttyUSB0"
#PORT = '/dev/ttyp5'
def main():
    """main"""
    logger = modbus_tk.utils.create_logger("console")

    try:
        #Connect to the slave
        master = modbus_rtu.RtuMaster(
            serial.Serial(port=PORT, baudrate=19200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
        )
        master.set_timeout(5.0)
        master.set_verbose(True)
        logger.info("connected")
        #
        logger.info(master.execute(4, cst.READ_HOLDING_REGISTERS, 0, 8))
        # print type(master.execute(4, cst.READ_HOLDING_REGISTERS, 0, 8))
        logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 1, output_value=6))#enable Climb Driver
        logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 282, output_value=1))  # enable Climb Driver
        logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 290, output_value=1))  # 10000 pulse 1 rpm,negtive up,positive up

        logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 97, output_value=1000))  # internal velocity
        logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 113, output_value=1000))  # internal velocity
        logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 114, output_value=1000))  # internal velocity
        logger.info(master.execute(4, cst.WRITE_SINGLE_REGISTER, 324, output_value=1000))  # set fixed velocity
        #
        logger.info(master.execute(4, cst.READ_HOLDING_REGISTERS, 212, 1))
        logger.info(master.execute(4, cst.READ_HOLDING_REGISTERS, 214, 1))
        logger.info(master.execute(4, cst.READ_HOLDING_REGISTERS, 218, 1))
        logger.info(master.execute(4, cst.READ_HOLDING_REGISTERS, 220, 1))
        # logger.info(master.execute(4, cst.READ_COILS, 0, 10))
        # logger.info(master.execute(4, cst.READ_DISCRETE_INPUTS, 0, 8))
        # logger.info(master.execute(4, cst.READ_INPUT_REGISTERS, 212, 3))
        logger.info(master.execute(4, cst.READ_HOLDING_REGISTERS, 212, 12))
        while 1:
            time.sleep(0.1)
            print master.execute(4, cst.READ_HOLDING_REGISTERS, 212, 12)
        # while True:
        #     if

    except modbus_tk.modbus.ModbusError as exc:
        logger.error("%s- Code=%d", exc, exc.get_exception_code())

if __name__ == "__main__":
    main()