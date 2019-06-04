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
            serial.Serial(port=PORT, baudrate=115200, bytesize=8, parity='O', stopbits=1, xonxoff=0)
        )
        master.set_timeout(5.0)
        master.set_verbose(True)
        logger.info("connected")
        #
        logger.info(master.execute(1, cst.READ_HOLDING_REGISTERS, 0, 8))
        # logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 8))
        # logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 0, 8))
        # #change to SigIn SON enable driver
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 3, output_value=1))
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 3, output_value=1))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 109, output_value=2))
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 109, output_value=2))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 112, output_value=50))
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 112, output_value=50))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 117, output_value=1))
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 117, output_value=1))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 120, output_value=120))#负逆时针
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 120, output_value=1000))#正的向下,负的向上
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 121, output_value=5000))
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 121, output_value=5000))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 128, output_value=100))
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 128, output_value=250))#下250,上500
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 69, output_value=1024))
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 69, output_value=1024))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 71, output_value=32767))
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 71, output_value=32767))
        # time.sleep(4)
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 71, output_value=31743))
        # logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 71, output_value=31743))
        # logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 4))
        #logger.info(master.execute(1, cst.READ_HOLDING_REGISTERS, 216, 1))
        # logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 18, 8))
        #change to SigIn SON enable driver
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 3, output_value=1))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 109, output_value=2))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 112, output_value=50))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 117, output_value=1))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 120, output_value=-1))#负逆时针
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 121, output_value=-5000))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 128, output_value=100))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 69, output_value=1024))
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 71, output_value=32767))
        # time.sleep(4)
        # logger.info(master.execute(2, cst.WRITE_SINGLE_REGISTER, 71, output_value=31743))
        # logger.info(master.execute(2, cst.READ_HOLDING_REGISTERS, 0, 4))


        # logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 8, 8))
        #change to SigIn SON enable driver
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 3, output_value=1))
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 109, output_value=2))
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 112, output_value=50))
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 117, output_value=1))
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 120, output_value=2000))#负上,正下
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 121, output_value=5000))
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 128, output_value=250))
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 69, output_value=1024))
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 71, output_value=32767))
        time.sleep(4)
        logger.info(master.execute(3, cst.WRITE_SINGLE_REGISTER, 71, output_value=31743))
        logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 0, 4))
        logger.info(master.execute(3, cst.READ_HOLDING_REGISTERS, 0, 4))
        #send some queries
        #logger.info(master.execute(2, cst.READ_COILS, 0, 10))
        #logger.info(master.execute(1, cst.READ_DISCRETE_INPUTS, 0, 8))
        #logger.info(master.execute(1, cst.READ_INPUT_REGISTERS, 100, 3))
        #logger.info(master.execute(1, cst.READ_HOLDING_REGISTERS, 100, 12))
        #logger.info(master.execute(1, cst.WRITE_SINGLE_COIL, 7, output_value=1))
        #logger.info(master.execute(1, cst.WRITE_SINGLE_REGISTER, 100, output_value=54))
        #logger.info(master.execute(1, cst.WRITE_MULTIPLE_COILS, 0, output_value=[1, 1, 0, 1, 1, 0, 1, 1]))
        #logger.info(master.execute(1, cst.WRITE_MULTIPLE_REGISTERS, 100, output_value=xrange(12)))

    except modbus_tk.modbus.ModbusError as exc:
        logger.error("%s- Code=%d", exc, exc.get_exception_code())

if __name__ == "__main__":
    main()