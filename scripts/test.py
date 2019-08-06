#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy
from std_msgs.msg import String,Float64

def main():
    topic = 'chatter'
    pub = rospy.Publisher(topic, String)
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo("I will publish to the topic %s", topic)
    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        rospy.sleep(0.1)
if __name__ == '__main__':
        main()