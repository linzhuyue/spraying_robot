#! /usr/bin/env python
# coding=utf-8
from ctypes import *
import yaml
import os
import logging
from logging.handlers import RotatingFileHandler
import os
from math import pi
from colorama import Fore, Style
from command import *
import time
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

class CanAnalysisDriver:
    def __init__(self,configname):
        self.configname=configname
        self.OpreateCanAnalysis = cdll.LoadLibrary('../lib/libcontrolcan.so')
        self.yamlDic={}
        self.Opreating_Yaml()# init yaml
        
        self.logger = logging.getLogger()## 创建一个logger
        self.logger_init()
    def Opreating_Yaml(self):
        current_path = os.path.abspath("..")
        yaml_path = os.path.join(current_path, "config/"+self.configname)
        print yaml_path
        file_data = open(yaml_path)
        self.yamlDic = yaml.load(file_data)
        file_data.close()
    def logger_init(self):
        # Log等级总开关
        self.logger.setLevel(logging.DEBUG)

        # 创建log目录
        if not os.path.exists('./logfiles'):
            os.mkdir('./logfiles')

        # 创建一个handler，用于写入日志文件
        logfile = './logfiles/mobilerobot-ctl-python.log'

        # 以append模式打开日志文件
        # fh = logging.FileHandler(logfile, mode='a')
        fh = RotatingFileHandler(logfile, mode='a', maxBytes=1024*1024*50, backupCount=30)

        # 输出到file的log等级的开关
        fh.setLevel(logging.INFO)

        # 再创建一个handler，用于输出到控制台
        ch = logging.StreamHandler()

        # 输出到console的log等级的开关
        ch.setLevel(logging.INFO)

        # 定义handler的输出格式
        # formatter = logging.Formatter("%(asctime)s - %(filename)s[line:%(lineno)d] - %(levelname)s: %(message)s")
        formatter = logging.Formatter("%(asctime)s [%(thread)u] %(levelname)s: %(message)s")

        # 为文件输出设定格式
        fh.setFormatter(formatter)

        # 控制台输出设定格式
        ch.setFormatter(formatter)

        # 设置文件输出到logger
        self.logger.addHandler(fh)

        # 设置控制台输出到logger
        self.logger.addHandler(ch)
    def loggererror(self,message):
        self.logger.error(Fore.RED + "[ERROR] - " + str(message) + Style.RESET_ALL)
    def loggerinfo(self,message):
        self.logger.info(Fore.GREEN + "[INFO] - " + str(message) + Style.RESET_ALL)
    def loggerwarning(self,message):
        self.logger.warning()(Fore.YELLOW + "[WARNNING] - " + str(message) + Style.RESET_ALL)
    def Init_node(self,):
        pass
    def Can_VCIOpenDevice(self):
        Canstatus=self.OpreateCanAnalysis.VCI_OpenDevice(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],0)
        if Canstatus==0:
            self.loggererror("Open Can Analysis device error")
            self.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.loggererror("Can analysis offline")
            self.loggerinfo(Canstatus)
        else:
            self.loggerinfo("Can Analysis Open Success!")
            self.loggerinfo(Canstatus)
            return True
    def Can_VCICloseDevice(self):
        Canstatus=self.OpreateCanAnalysis.VCI_CloseDevice(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'])
        if Canstatus==0:
            self.loggererror("Close can analysis device error!")
            self.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggerinfo(Canstatus)
        else:
            self.loggerinfo("Can Analysis Close Success!!")
            self.loggerinfo(Canstatus)
            return True  
    def Can_VCIInitCan_PyInit(self,CanInd,VCI_INIT_CONFIG_STRUC):
        
        CanFuncStruc=self.OpreateCanAnalysis.VCI_InitCAN#(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        CanFuncStruc.restype = c_uint
        CanFuncStruc.argtypes=[c_uint,c_uint,c_uint,POINTER(VCI_INIT_CONFIG)]
        return CanFuncStruc(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,byref(VCI_INIT_CONFIG_STRUC))
    def Can_VCIInitCan(self,CanInd):
        config=VCI_INIT_CONFIG()
        config.AccCode = 0x00000000
        config.AccMask = 0xFFFFFFFF
        config.Filter = 0
        config.Mode = 0
        config.Timing0 = 0x00
        config.Timing1 = 0x1c
        #print(config)
        Canstatus=self.Can_VCIInitCan_PyInit(CanInd,config)
        if Canstatus==0:
            self.loggererror("Init Can analysis device error!")
            self.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggerinfo(Canstatus)
        else:
            self.loggerinfo("Can Analysis Init Success!!")
            self.loggerinfo(Canstatus)
            return True   
    def Can_ReadBoardInfo_PyInit(self,VCI_BOARD_INFO_STRUC):
        CanFuncStruc=self.OpreateCanAnalysis.VCI_ReadBoardInfo#(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        CanFuncStruc.restype = c_uint
        CanFuncStruc.argtypes=[c_uint,c_uint,POINTER(VCI_BOARD_INFO)]
        return CanFuncStruc(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],byref(VCI_BOARD_INFO_STRUC))
    def Can_ReadBoardInfo(self):
        config=VCI_BOARD_INFO()
        Canstatus=self.Can_ReadBoardInfo_PyInit(config)

        print(config.can_Num)
        if Canstatus==0:
            self.loggererror("Read Board Info Can analysis device error!")
            self.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggerinfo(Canstatus)
        else:
            self.loggerinfo("Can Analysis Read Board Info Success!!")
            self.loggerinfo(Canstatus)
            self.loggerinfo("Can Analysis Board Info like:")
            self.loggerinfo("hw_Version: "+str(config.hw_Version))
            self.loggerinfo("fw_Version: "+str(config.fw_Version))
            self.loggerinfo("dr_Version: "+str(config.dr_Version))
            self.loggerinfo("in_Version: "+str(config.in_Version))
            self.loggerinfo("irq_Num: "+str(config.irq_Num))
            self.loggerinfo("can_Num: "+str(config.can_Num))
            self.loggerinfo("str_Serial_Num: "+str(config.str_Serial_Num))
            self.loggerinfo("str_hw_Type: "+str(config.str_hw_Type))
            self.loggerinfo("Reserved: "+str(config.Reserved))

            return Canstatus 
    def Can_GetReceiveNum(self,CanInd):
        Canstatus=self.OpreateCanAnalysis.VCI_GetReceiveNum(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd)
        if Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggerinfo(Canstatus)
        elif Canstatus==0:
            self.loggererror("Can Analysis Get Receive Num No data")
            self.loggererror(Canstatus)
        else:
            self.loggerinfo("Can Analysis Get Receive Num Success!!")
            self.loggerinfo(Canstatus)
            return Canstatus  
    def Can_ClearBuffer(self,CanInd):
        Canstatus=self.OpreateCanAnalysis.VCI_ClearBuffer(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        if Canstatus==0:
            self.loggererror("Clear Buffer Can analysis device error!")
            self.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggerinfo(Canstatus)
        else:
            self.loggerinfo("Can Analysis Clear Buffer Success!!")
            self.loggerinfo(Canstatus)
            return True  
    def Can_StartCAN(self,CanInd):
        Canstatus=self.OpreateCanAnalysis.VCI_StartCAN(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        if Canstatus==0:
            self.loggererror("Start Can analysis device error!")
            self.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggerinfo(Canstatus)
        else:
            self.loggerinfo("Can Analysis Start Success!!")
            self.loggerinfo(Canstatus)
            return True  
    def Can_ResetCAN(self,CanInd):
        Canstatus=self.OpreateCanAnalysis.VCI_ResetCAN(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        if Canstatus==0:
            self.loggererror("Reset Can analysis device error!")
            self.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggerinfo(Canstatus)
        else:
            self.loggerinfo("Can Analysis Reset Success!!")
            self.loggerinfo(Canstatus)
            return True  
    def Can_Transmit_PyInit(self,CanInd,Length,VCI_CAN_OBJ_STRUC):
        CanFuncStruc=self.OpreateCanAnalysis.VCI_Transmit#(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        CanFuncStruc.restype = c_uint
        CanFuncStruc.argtypes=[c_uint,c_uint,c_uint,POINTER(VCI_CAN_OBJ),c_uint]
        return CanFuncStruc(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,byref(VCI_CAN_OBJ_STRUC),Length)
    def Can_Transmit(self,CanInd,Length,IDD,Datalen,SenData,):
        #for i in range(Length):
        config=VCI_CAN_OBJ()
        config.ID=IDD
        config.RemoteFlag=0
        config.ExternFlag =0
        config.SendType = 0
        config.DataLen = Datalen
        config.Data=SenData
        # ubyte_array = c_ubyte*8
        # a = ubyte_array(0x04,0x01,0x01,0x00)

        # ubyte_3array = c_ubyte*3
        # b = ubyte_3array(0, 0 , 0)
        # vci_can_obj_2 = VCI_CAN_OBJ(1, 0, 0, 0, 0, 0,  8, a, b)
        # print(SenData)
        # for i in range(0,Datalen):
        #     # print(type(SenData[i]))
        #     config.Data[i]=SenData[i]
            
        self.loggerinfo(SenData)
        #len+=1

        Canstatus=self.Can_Transmit_PyInit(0,8,config)
        time.sleep(0.1)
        if Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggererror(Canstatus)
        elif Canstatus==0:
            self.loggererror("Can analysis Transmit No data")
            self.loggererror(Canstatus)
        else:
            self.loggerinfo("Can Analysis Data Transmit Success!!")
            self.loggerinfo(Canstatus)
            return Canstatus  
    def Can_Receive_PyInit(self,CanInd,Len,WaitTime,VCI_CAN_OBJ_STRUC):
        CanFuncStruc=self.OpreateCanAnalysis.VCI_Receive#(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,)
        CanFuncStruc.restype = c_uint
        CanFuncStruc.argtypes=[c_uint,c_uint,c_uint,POINTER(VCI_CAN_OBJ),c_ulong,c_int]
        return CanFuncStruc(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],CanInd,byref(VCI_CAN_OBJ_STRUC),Len,WaitTime)

 
    def Can_Receive(self,CanInd,Len):
        config=VCI_CAN_OBJ()
        # ubyte_array = c_ubyte*8
        # a = ubyte_array(0, 0, 0, 0, 0, 0, 0, 0)
        # ubyte_3array = c_ubyte*3
        # b = ubyte_3array(0, 0 , 0)
        # config = VCI_CAN_OBJ(0x0, 0, 0, 0, 0, 0,  8, a, b)

        Canstatus=self.Can_Receive_PyInit(CanInd,Len,0,config)
        if Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggererror(Canstatus)
        elif Canstatus==0:
            self.loggererror("Can Analysis Receive No data!!")
            self.loggererror(Canstatus)
        else:
            self.loggerinfo("Can Analysis Receive Success!!")
            self.loggerinfo(Canstatus)
            return Canstatus,config
    def Can_VCI_UsbDeviceReset(self):
        Canstatus=self.OpreateCanAnalysis.VCI_UsbDeviceReset(self.yamlDic['nDeviceType'],self.yamlDic['nDeviceInd'],0)
        if Canstatus==0:
            self.loggererror("Reset USB Can analysis device error!")
            self.loggerinfo(Canstatus)
        elif Canstatus== -1:
            self.loggererror("Can analysis offline!")
            self.loggerinfo(Canstatus)
        else:
            self.loggerinfo("Can Analysis Reset USB Success!!")
            self.loggerinfo(Canstatus)
            return True  
def main():
    configname="mobileparameter.yaml"
    md=CanAnalysisDriver(configname)
    # md.Opreating_Yaml()
    md.loggerinfo(md.yamlDic)
    
    md.Can_VCIOpenDevice()
    md.Can_VCIInitCan(0)
    md.Can_StartCAN(0)
    md.Can_ResetCAN(0)
    md.Can_ReadBoardInfo()
    comd=MobileDriverCommands()
    md.Can_Transmit(0,1,1,8,comd.REQUEST_ENCODER_1)
    time.sleep(0.3)
    renum=md.Can_GetReceiveNum(0)
    ret,kk=md.Can_Receive(0,1)

    while renum:
        
        transmit_status=md.Can_Transmit(0,1,1,8,comd.REQUEST_ENCODER_1)
        if transmit_status:
            # time.sleep(0.1)
            ret,kk=md.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                    print('my data 1',list(kk.Data))
                    print(kk.DataLen)
        else:
            print('my data 2',list(kk.Data))
            print(kk.DataLen)
        transmit_status=md.Can_Transmit(0,1,2,8,comd.REQUEST_ENCODER_2)
        if transmit_status:
            # time.sleep(0.1)
            ret,kk=md.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                    print('my data 2',list(kk.Data))
                    print(kk.DataLen)
        else:
            print('my data 2',list(kk.Data))
            print(kk.DataLen)
        transmit_status=md.Can_Transmit(0,1,3,8,comd.REQUEST_ENCODER_3)
        if transmit_status:
            # time.sleep(0.1)
            ret,kk=md.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                    print('my data 3',list(kk.Data))
                    print(kk.DataLen)
        else:
            print('my data 3',list(kk.Data))
            print(kk.DataLen)
        transmit_status=md.Can_Transmit(0,1,4,8,comd.REQUEST_ENCODER_4)
        if transmit_status:
            # time.sleep(0.1)
            ret,kk=md.Can_Receive(0,1)
            if ret:
                if list(kk.Data)[0]!=0 and list(kk.Data)[0]!=127:
                    print('my data 4',list(kk.Data))
                    print(kk.DataLen)
        else:
            print('my data 4',list(kk.Data))
            print(kk.DataLen)
        # time.sleep(0.3)
        # if ret>0:
        #     print('my data 1',list(kk.Data))
        #     print(kk.DataLen)
       
        # md.Can_Transmit(0,1,2,8,comd.REQUEST_ENCODER_2)
        # time.sleep(0.3)
        # ret,kk2=md.Can_Receive(0,1)
        # time.sleep(0.3)
        # if ret>0:
        #     print('my data 2',list(kk2.Data))
        #     print(kk2.DataLen)
        # time.sleep(0.3)
        # md.Can_Transmit(0,1,3,8,comd.REQUEST_ENCODER_3)
        
        # ret,kk3=md.Can_Receive(0,1)
        # time.sleep(0.3)
        # if ret>0:
        #     print('my data 3',list(kk3.Data))
        #     print(kk3.DataLen)
        
        # md.Can_Transmit(0,1,4,8,comd.REQUEST_ENCODER_4)
        # time.sleep(0.3)
        # ret,kk4=md.Can_Receive(0,1)
        # if ret>0:
        #     print('my data 4',list(kk4.Data))
        #     print(kk.DataLen)
        
    # time.sleep(3)
    md.Can_VCICloseDevice()
    # md.OpreateCanAnalysis.VCI_CloseDevice(4, 0,0)
if __name__=="__main__":
    main()