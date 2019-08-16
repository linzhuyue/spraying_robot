#! /usr/bin/env python
# coding=utf-8
from ctypes import *
import time
dll =cdll.LoadLibrary('../lib/libcontrolcan.so')
nDeviceType = 4 #* USBCAN-2E-U *
nDeviceInd = 0#* 索引号0 *
nReserved = 0
nCANInd = 0 #can通道号

class _VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_ulong), ("AccMask", c_ulong), ("Reserved", c_ulong), ("Filter", c_ubyte),
                ("Timing0", c_ubyte), ("Timing1", c_ubyte), ("Mode", c_ubyte)]
class _VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint), ("TimeStamp", c_uint), ("TimeFlag", c_ubyte), ("SendType", c_ubyte),
                ("RemoteFlag", c_ubyte), ("ExternFlag", c_ubyte), ("DataLen", c_ubyte), ("Data", c_ubyte*8),
                ("Reserved", c_ubyte*3)]
class VCI_BOARD_INFO(Structure):
    _fields_ = [('hw_Version',c_ushort),
                ('fw_Version',c_ushort),
                ('dr_Version',c_ushort),
                ('in_Version',c_ushort),
                ('irq_Num',c_ushort),
                ('can_Num',c_byte),
                ('str_Serial_Num',c_char*20),
                ('str_hw_Type',c_char*40),
                ('Reserved',c_ushort*4)
                                        ]

vic = _VCI_INIT_CONFIG()
vic.AccCode = 0x00000000
vic.AccMask = 0xffffffff
vic.Filter = 0
vic.Timing0 = 0x00
vic.Timing1 = 0x1c
vic.Mode = 0

vco = _VCI_CAN_OBJ()
vco.ID = 0x00000001
vco.SendType = 0
vco.RemoteFlag = 0
vco.ExternFlag = 0
vco.DataLen = 8
vco.Data = (0x04,0x01,0x01,0x00)
# vco.Data = (4,1,1,0,0,0,0,0)

vco2 = _VCI_CAN_OBJ()

#ubyte_array8 = c_ubyte*8
#ubyte_array3 = c_ubyte*3
# vco2.ID = 0x00000001
# vco2.SendType = 0
# vco2.RemoteFlag = 0
# vco2.ExternFlag = 0
# vco2.DataLen = 8
# vco2.Data = (0, 0, 0, 0, 0, 0, 0, 0)

ret = dll.VCI_OpenDevice(nDeviceType, nDeviceInd, nReserved)
print("opendevice:",ret)
time.sleep(1)
# ret = dll.VCI_SetReference(nDeviceType, nDeviceInd, 0, 0, pointer(c_int(0x060007)))
# print("setrefernce1:",ret)
# time.sleep(1)
# ret = dll.VCI_SetReference(nDeviceType, nDeviceInd, 0, 0, byref(c_int(0x060003)))
# print("setrefernce0:",ret)  #注意，SetRefernce必须在InitCan之前
ret = dll.VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, byref(vic))
print("initcan1:",ret)
time.sleep(1)
# class VCI_BOARD_INFO(Structure):
kk=VCI_BOARD_INFO()
ret = dll.VCI_ReadBoardInfo(4,0,byref(kk))
print("board",ret)
print(kk.str_Serial_Num)
print(kk.str_hw_Type)
# ret = dll.VCI_InitCAN(nDeviceType, nDeviceInd, 0, byref(vic))
# print("initcan0:",ret)

# ret = dll.VCI_StartCAN(nDeviceType, nDeviceInd, nCANInd)
# print("startcan1:",ret)
ret = dll.VCI_StartCAN(nDeviceType, nDeviceInd, 0)
time.sleep(1)
print("startcan0:",ret)
while 1:
    time.sleep(1)
    ret = dll.VCI_Transmit(nDeviceType, nDeviceInd, nCANInd, byref(vco), 1)# 发送两帧
    print("transmit:",ret)
    time.sleep(1)
    ret = dll.VCI_GetReceiveNum(4,0,0)
    print("get receive num:",ret)
    ret = dll.VCI_Receive(nDeviceType, nDeviceInd, 0, byref(vco2), 1, 0)# 发送两帧
    print("receive:",ret)
    if ret > 0:
        print(vco2.DataLen)
        print(list(vco2.Data))

