#!/usr/bin/env python
# -*- coding: utf-8 -*-
from pid_control import PID
import time
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline
#这个程序的实质就是在前九秒保持零输出，在后面的操作中在传递函数为某某的系统中输出1

def test_pid(P = 0.2,  I = 0.0, D= 0.0, L=100):
    """Self-test PID class

    .. note::
        ...
        for i in range(1, END):
            pid.update(feedback)
            output = pid.output
            if pid.SetPoint > 0:
                feedback += (output - (1/i))
            if i>9:
                pid.SetPoint = 1
            time.sleep(0.02)
        ---
    """
    pid = PID(P, I, D)

    pid.SetPoint=0.0
    pid.setSampleTime(0.01)

    END = L
    feedback = 0

    feedback_list = []
    time_list = []
    setpoint_list = []

    for i in range(1, END):
        pid.update(feedback)
        output = pid.output
        if pid.SetPoint > 0:
            feedback +=output# (output - (1/i))控制系统的函数
        if i>9:
            pid.SetPoint = 10
        time.sleep(0.01)

        feedback_list.append(feedback)
        setpoint_list.append(pid.SetPoint)
        time_list.append(i)

    time_sm = np.array(time_list)
    time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
    feedback_smooth = spline(time_list, feedback_list, time_smooth)
    plt.figure(0)
    plt.plot(time_smooth, feedback_smooth)
    plt.plot(time_list, setpoint_list)
    plt.xlim((0, L))
    plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
    plt.xlabel('time (s)')
    plt.ylabel('PID (PV)')
    plt.title('TEST PID')

    plt.ylim((1-10.5, 1+10.5))
    # plt.ylim((1-0.5, 1+0.5))
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    test_pid(1.2, 1, 0.001, L=80)
#    test_pid(0.8, L=50)