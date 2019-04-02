#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time
# from tf.transformations import
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped,Pose
from nav_msgs.msg import Path

class MvURTest():
    def __init__(self):
        self.ee_pose_buf=[]
        self.ee_pub=rospy.Publisher("/ee_trajectory",Path,queue_size=10)
        self.ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
    def Init_node(self):
        rospy.init_node("move_ur5_by_urscript")
        #
    def tf_callback(self,msg):
        # print msg.transforms[0],type(msg.transforms[0])
        if len(self.ee_pose_buf)==10:
            self.ee_pose_buf=self.ee_pose_buf[1:]
            if msg.transforms[0].child_frame_id =="tool0_controller":
                translation_x=msg.transforms[0].transform.translation.x
                translation_y = msg.transforms[0].transform.translation.y
                translation_z = msg.transforms[0].transform.translation.z
                rotation_quaternion_x=msg.transforms[0].transform.rotation.x
                rotation_quaternion_y = msg.transforms[0].transform.rotation.y
                rotation_quaternion_z = msg.transforms[0].transform.rotation.z
                rotation_quaternion_w = msg.transforms[0].transform.rotation.w
                self.ee_pose_buf.append([translation_x,translation_y,translation_z,rotation_quaternion_x,rotation_quaternion_y,rotation_quaternion_z,rotation_quaternion_w])
        else:
            if msg.transforms[0].child_frame_id == "tool0_controller":
                translation_x=msg.transforms[0].transform.translation.x
                translation_y = msg.transforms[0].transform.translation.y
                translation_z = msg.transforms[0].transform.translation.z
                rotation_quaternion_x=msg.transforms[0].transform.rotation.x
                rotation_quaternion_y = msg.transforms[0].transform.rotation.y
                rotation_quaternion_z = msg.transforms[0].transform.rotation.z
                rotation_quaternion_w = msg.transforms[0].transform.rotation.w
                self.ee_pose_buf.append(
                    [translation_x, translation_y, translation_z, rotation_quaternion_x, rotation_quaternion_y,
                     rotation_quaternion_z, rotation_quaternion_w])

        # print msg.transforms
    def change_angle_to_pi(self,qangle):
        temp=[]
        for i in xrange(len(qangle)):
            temp.append(qangle[i]/180.0*3.14)
        return temp
    def moveur(self,q,ace,vel,t):
        ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
        print ss
        self.ur_pub.publish(ss)
        return ss

"""

rosrun rosserial_python serial_node.py /dev/ttyUSB0
rostopic pub /toggle_led std_msgs/Empty "{}" --once
"""
def main():
    t = 0
    vel=0.1
    ace=50
    # vel = 1.05
    # ace = 1.4
    urv=MvURTest()
    urv.Init_node()
    qq = [-45, -180, 90, -180, -90, 180]
    rate = rospy.Rate(0.5)
    qt = urv.change_angle_to_pi(qq)
    reference_frame="base"
    tf_sub = rospy.Subscriber("/tf", TFMessage, urv.tf_callback)
    while not rospy.is_shutdown():

        if len(urv.ee_pose_buf)!=0:
            print "now_ee_pose_in_base_frame",urv.ee_pose_buf[-1]
            path=Path()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = "tool0_controller"

            this_pose_stamped=PoseStamped()
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id = "tool0_controller"
            this_pose_stamped.pose.position.x = urv.ee_pose_buf[-1][0]#x
            print "this_pose_stamped.pose.position.x",this_pose_stamped.pose.position.x
            this_pose_stamped.pose.position.y = urv.ee_pose_buf[-1][1]#y
            this_pose_stamped.pose.position.z = urv.ee_pose_buf[-1][2]  # z
            this_pose_stamped.pose.orientation.x = urv.ee_pose_buf[-1][3]
            this_pose_stamped.pose.orientation.y = urv.ee_pose_buf[-1][4]
            this_pose_stamped.pose.orientation.z = urv.ee_pose_buf[-1][5]
            this_pose_stamped.pose.orientation.w = urv.ee_pose_buf[-1][6]
            path.poses.append(this_pose_stamped)
            urv.ee_pub.publish(path)
            # urv.moveur(qt,ace,vel,t)
        rate.sleep()
        # rospy.spin()
if __name__=="__main__":
    main()