#! /usr/bin/env python
# coding=utf-8
import math
from robotcontrol import *
import rospy
from std_msgs.msg import String,Float64,Bool
class Test():
    def __init__(self):
        self.OpenstateBool=[False]
        rospy.init_node("move_aubo_trajectory_planning_test")
        self.PubState = rospy.Publisher("/open_aubo_state_flag", Bool, queue_size=10)

def main():
    ak=Test()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        ak.PubState.publish(False)
        print "publish-----"
        rate.sleep()

if __name__=="__main__":
    main()
    # print deg_to_rad((-3.3364,12.406,-81.09,-91.207,-86.08,0.164))