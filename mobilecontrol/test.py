#! /usr/bin/env python
# coding=utf-8
from ctypes import *
import time 
VCI_USBCAN2A = 4
STATUS_OK = 1
# class VCI_INIT_CONFIG(Structure):  
#     _fields_ = [("AccCode", c_ulong),
#                 ("AccMask", c_ulong),
#                 ("Reserved", c_ulong),
#                 ("Filter", c_ubyte),
#                 ("Timing0", c_ubyte),
#                 ("Timing1", c_ubyte),
#                 ("Mode", c_ubyte)
#                 ]  
# class VCI_CAN_OBJ(Structure):  
#     _fields_ = [("ID", c_uint),
#                 ("TimeStamp", c_uint),
#                 ("TimeFlag", c_ubyte),
#                 ("SendType", c_ubyte),
#                 ("RemoteFlag", c_ubyte),
#                 ("ExternFlag", c_ubyte),
#                 ("DataLen", c_ubyte),
#                 ("Data", c_ubyte*8),
#                 ("Reserved", c_ubyte*3)
#                 ] 
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
class VCI_CAN_OBJ(Structure):
    _fields_ = [('ID',c_uint),
                ('TimeStamp',c_uint),
                ('TimeFlag',c_byte),
                ('SendType',c_byte),
                ('RemoteFlag',c_byte),
                ('ExternFlag',c_byte),
                ('DataLen',c_byte),
                ('Data',c_ubyte*8),
                ('Reserved',c_ubyte*3)
                                        ]
class VCI_INIT_CONFIG(Structure):
    _fields_ = [('AccCode',c_uint),
                ('AccMask',c_uint),
                ('Reserved',c_uint),
                ('Filter',c_ubyte),
                ('Timing0',c_ubyte),
                ('Timing1',c_ubyte),
                ('Mode',c_ubyte)
                                        ] 
canDLL = cdll.LoadLibrary('../lib/libcontrolcan.so')
# canDLL = windll.LoadLibrary(CanDLLName)
# print(CanDLLName)
 
ret = canDLL.VCI_OpenDevice(VCI_USBCAN2A, 0, 0)
print(ret)
if ret != STATUS_OK:
    print('调用 VCI_OpenDevice出错\r\n')
 
#初始0通道
vci_initconfig = VCI_INIT_CONFIG(0x00000000, 0xFFFFFFFF, 0,0, 0x00, 0x1C, 0)
ret = canDLL.VCI_InitCAN(4, 0, 0, byref(vci_initconfig))
if ret != STATUS_OK:
    print('调用 VCI_InitCAN出错\r\n')
 
ret = canDLL.VCI_StartCAN(VCI_USBCAN2A, 0, 0)
if ret != STATUS_OK:
    print('调用 VCI_StartCAN出错\r\n')
 
# #初始1通道
# ret = canDLL.VCI_InitCAN(VCI_USBCAN2A, 0, 1, byref(vci_initconfig))
# if ret != STATUS_OK:
#     print('调用 VCI_InitCAN 1 出错\r\n')
 
# ret = canDLL.VCI_StartCAN(VCI_USBCAN2A, 0, 1)
# if ret != STATUS_OK:
#     print('调用 VCI_StartCAN 1 出错\r\n')
 
#通道0发送数据
ubyte_array = c_ubyte*8
a = ubyte_array(0x04,0x01,0x01,0x00)

ubyte_3array = c_ubyte*3
b = ubyte_3array(0, 0 , 0)
vci_can_obj_1 = VCI_CAN_OBJ(1, 0, 0, 0, 0, 0,  8, a, b)
 
ret = canDLL.VCI_Transmit(4, 0, 0, byref(vci_can_obj_1), 1)
print("i send data",ret)
if ret != STATUS_OK:
    print('调用 VCI_Transmit 出错\r\n')
 
#通道1接收数据
time.sleep(1)
a = ubyte_array(0, 0, 0, 0, 0, 0, 0, 0)
vci_can_obj = VCI_CAN_OBJ(0x0, 0, 0, 0, 0, 0,  8, a, b)
# elems = (POINTER(VCI_CAN_OBJ) * 2500)()
# vci_can_obj_arrar=cast(elems,POINTER(POINTER(VCI_CAN_OBJ)))
# ret = canDLL.VCI_Receive(VCI_USBCAN2A, 0, 0, byref(vci_can_obj_arrar), 2500, 0)
# print("data",ret)
ret = canDLL.VCI_Receive(VCI_USBCAN2A, 0, 0, byref(vci_can_obj), 1, 0)
while ret <= 0:
    print('调用 VCI_Receive 出错\r\n')
    ret = canDLL.VCI_Transmit(VCI_USBCAN2A, 0, 0, byref(vci_can_obj_1), 1)
    print("i send data",ret)
    if ret != STATUS_OK:
        print('调用 VCI_Transmit 出错\r\n')
    ret = canDLL.VCI_Receive(VCI_USBCAN2A, 0, 0, byref(vci_can_obj), 1, 0)
    print ret
    time.sleep(0.3)
# a = ubyte_array(0, 0, 0, 0, 0, 0, 0, 0)
# vci_can_obj = VCI_CAN_OBJ(0x0, 0, 0, 0, 0, 0,  8, a, b)
while 1:
    if ret > 0:
        print("I receive data la",ret)
        print(vci_can_obj.DataLen)
        print('my data',list(vci_can_obj.Data))
        for i in list(vci_can_obj.Data):
            print "i",hex(i)
    time.sleep(0.5)
#关闭
canDLL.VCI_CloseDevice(VCI_USBCAN2A, 0) 