#!/usr/bin/env python
import rospy
# from ur5_planning.msg import structure_point
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
# geometry_msgs/TwistStamped
from geometry_msgs.msg import TwistStamped
import time
class UrToolVelocityRead():
    def __init__(self):
       # self.nodename=nodename
        self.Ur_tool_velocity_buf=[]



    def Init_node(self):
        rospy.init_node("Get_tool_velocity_node")
        tool_sub = rospy.Subscriber("/tool_velocity", TwistStamped, self.Ur_tool_velocity_callback)

        return tool_sub
    def Ur_tool_velocity_callback(self,msg):
        # print "msg",msg
        if len(self.Ur_tool_velocity_buf)==10:
            self.Ur_tool_velocity_buf=self.Ur_tool_velocity_buf[1:]
            xd=msg.twist.linear.x
            yd=msg.twist.linear.y
            zd = msg.twist.linear.z
            self.Ur_tool_velocity_buf.append([xd,yd,zd])
            #print "---------self.cross_uv_buf",self.cross_uv_buf
        else:
            xd=msg.twist.linear.x
            yd=msg.twist.linear.y
            zd = msg.twist.linear.z
            self.Ur_tool_velocity_buf.append([xd,yd,zd])


def main():
    uv0=UrToolVelocityRead()
    sub=uv0.Init_node()
    while not rospy.is_shutdown():
        # if len(uv0.Ur_tool_velocity_buf)==0:
        #     print "wait data----\n"
        #     pass
        # else:
        #     time.sleep(1)

        print "sturucture_point_xd_buf------",uv0.Ur_tool_velocity_buf
        time.sleep(1)
        #uv0.uvlist=[]
    #rospy.spin()
if __name__=="__main__":
    main()