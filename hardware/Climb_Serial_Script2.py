#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time
import binascii
s=serial.Serial('/dev/ttyUSB0',19200,bytesize=8, parity='O', stopbits=1,timeout=None, xonxoff=0)#

message_bytes = "030300000002c5e9".decode('hex')#"0306000000008828".decode('hex')#8828#03060000000149e8
# print str(message_bytes)
s.write(message_bytes)
s.flushInput()
s.flushOutput()
time.sleep(0.1)
# print s.read(1)
# print bytes(s.read(15))#.encode('hex')
# print binascii.hexlify(s.read(15))

serial_data =''
while s.inWaiting() > 0:
    c=s.read(1)
    # or    c=ser.read(1).decode('latin1')
    serial_data += c
    s.flushOutput()
# print serial_data.encode('utf8')
print binascii.hexlify(serial_data)#.decode('hex')
# message_bytes_2='03060002ff9c6871'.decode("hex")
# # kk="\x01\x06\x00\x02\x03\xE8\x28\xB4"
# s.write(message_bytes_2)
# time.sleep(0.01)
#
# print s.read(15).encode('hex')
# s.reset_output_buffer()
# message_bytes_2='030300000002c5e9'.decode("hex")
# # kk="\x01\x06\x00\x02\x03\xE8\x28\xB4"
# print s.write(message_bytes_2)
# time.sleep(0.1)
#
# print s.read(15).replace(' ','').encode('hex')
# print s.read().encode('hex')
# print s.read().encode('hex')
# print s.read().encode('hex')
# print s.read().encode('hex')
# print s.read().encode('hex')
# print s.read().encode('hex')
# print s.read().encode('hex')
s.close()