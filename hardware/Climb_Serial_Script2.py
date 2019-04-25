#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import time
s=serial.Serial('/dev/ttyUSB0',19200,bytesize=8, parity='O', stopbits=1, xonxoff=0)

message_bytes = "010600000001480A".decode('hex')
print str(message_bytes)
s.write(message_bytes)
time.sleep(0.1)
message_bytes_2='0106000203E828B4'.decode("hex")
# kk="\x01\x06\x00\x02\x03\xE8\x28\xB4"
s.write(message_bytes_2)
time.sleep(0.1)

print s.read(20)
s.close()