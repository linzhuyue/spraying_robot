#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from sensor_msgs.msg import JointState


class UrSimposition():

    def __init__(self, name = "ur_info_subscriber" ):

        self.name = name
        self.joint_states_pub=rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.arm_joints = ['mr_steeringjoint01',
                           'mr_drivingjoint01',
                           'mr_steeringjoint02',
                           'mr_drivingjoint02',
                           'mr_steeringjoint03',
                           'mr_drivingjoint03',
                           'mr_steeringjoint04',
                           'mr_drivingjoint04',
                           'cr_joint00',
                           'cr_joint01',
                           'cr_joint02',
                           'ur_joint01',
                           'ur_joint02',
                           'ur_joint03',
                           'ur_joint04',
                           'ur_joint05',
                           'ur_joint06']
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
        joint_info.header.frame_id = "mr_body"
        joint_info.header.seq=cn
        joint_info.name=self.arm_joints
        joint_info.position=pose
        joint_info.velocity=[0.1 for i in self.arm_joints]
        joint_info.effort=[]
        self.joint_states_pub.publish(joint_info)
    def just_ur5(self,pose):
        kk=[0.0 for i in self.arm_joints]
        kk[10:]=pose
        return kk

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
        joint_all=ur_info.just_ur5(angular_to_pi)
        print joint_all
        ur_info.set_position(joint_all,cn)
        cn+=1
        rate.sleep()

if __name__ == "__main__":
    main()