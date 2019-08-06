#! /usr/bin/env python
# coding=utf-8
import time
from ctypes import cdll

cur = cdll.LoadLibrary('../lib/libcontrolcan.so')
kk=cur.VCI_OpenDevice(4,0,0)
if kk !=1:
    print("Open error")