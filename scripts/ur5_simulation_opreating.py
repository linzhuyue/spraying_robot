#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from sensor_msgs.msg import JointState


class UrSimposition():

    def __init__(self, name = "ur_info_subscriber" ):

        self.name = name
        self.joint_states_pub=rospy.Publisher("/joint_states", JointState, queue_size=10)

    def Init_node(self):
        rospy.init_node(self.name)
    def change_angle_to_pi(self,qangle):
        temp = []
        for i in xrange(len(qangle)):
            temp.append(qangle[i] / 180.0 * 3.14)
        return temp
    def set_position(self,pose,cn):
        joint_info=JointState()
        joint_info.header.stamp = rospy.Time.now()
        joint_info.header.frame_id = "base"
        joint_info.header.seq=cn
        joint_info.name=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
               "wrist_3_joint"]
        joint_info.position=pose
        joint_info.velocity=[]
        joint_info.effort=[]
        self.joint_states_pub.publish(joint_info)

def main():
    ur_info = UrSimposition()
    ur_info.Init_node()
    qq=[-85, -180, 90, -180, -90, 180]
    qq1=[-15, -180, 90, -180, -90, 180]
    angular_to_pi=ur_info.change_angle_to_pi(qq)
    print "angular_to_pi",angular_to_pi
    cn=0
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        ur_info.set_position(angular_to_pi,cn)
        cn+=1
        rate.sleep()

if __name__ == "__main__":
    main()