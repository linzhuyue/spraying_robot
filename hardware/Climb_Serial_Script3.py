#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import serial.rs485
import time
from textwrap import wrap
import binascii
import struct
s=serial.Serial('/dev/ttyUSB0',19200,bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=1,timeout=0.3, xonxoff=0,rtscts=False,dsrdtr=False)#
# s.rs485_mode = serial.rs485.RS485Settings(rts_level_for_tx=True, rts_level_for_rx=False, loopback=False, delay_before_tx=None, delay_before_rx=None)
print serial.__version__
message_bytes = "03060000000149e8".decode('hex')#"0306000000008828".decode('hex')#8828#03060000000149e8
# print str(message_bytes)
s.write(message_bytes)
s.flushInput()
s.flushOutput()
time.sleep(0.1)
# print s.read(1)

print bytes(s.read(25)).encode('hex')
# print binascii.hexlify(s.read(15))

# serial_data =''
# while s.in_waiting() > 0:
#     c=s.read(1)
#     # or    c=ser.read(1).decode('latin1')
#     serial_data += c
#     s.flushOutput()
#     print hex(int(c.encode('hex'),16))
#     # w = struct.unpack("h", c)[0]
#     # print w
#     # print binascii.hexlify(c)
#     # print int(c,16)
# print serial_data.encode('hex')
# print binascii.hexlify(serial_data.replace('\n',''))#.decode('hex')

message_bytes_2='030300000002c5e9'.decode("hex")#030300000002c5e9#03060002ff9c6871
# kk="\x01\x06\x00\x02\x03\xE8\x28\xB4"
s.write(message_bytes_2)
s.flushInput()
s.flushOutput()
# serial_data =''
# while s.in_waiting() > 0:
#     c=s.read(1)
#     # or    c=ser.read(1).decode('latin1')
#     serial_data += c
#     s.flushOutput()
#     print hex(int(c.encode('hex'),16))
#     # w = struct.unpack("h", c)[0]
#     # print w
#     # print binascii.hexlify(c)
#     # print int(c,16)
# print serial_data.encode('hex')
# print binascii.hexlify(serial_data.replace('\n',''))#.decode('hex')

time.sleep(0.01)
strt=s.read(25).encode('hex')
# print (s.read(25)[2:].zfill(2)).encode('hex')
# print bytes(s.read(25)).encode('hex')
print str(strt)


print wrap(strt,2)

s.close()