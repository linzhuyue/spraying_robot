#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time
rospy.init_node("move_ur5_by_urscript")
pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
rate=rospy.Rate(1)
def change_angle_to_pi(qangle):
    temp=[]
    for i in xrange(len(qangle)):
        temp.append(qangle[i]/180.0*3.14)
    return temp
def moveur(pub,q,ace,vel,t):
    ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    print ss
    pub.publish(ss)
def movelur(pub,q,ace,vel,t):
    ss="movel(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    print ss
    pub.publish(ss)
def movecur(pub,q,ace,vel,t):
    ss="movec(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    print ss
    pub.publish(ss)
"""

rosrun rosserial_python serial_node.py /dev/ttyUSB0
rostopic pub /toggle_led std_msgs/Empty "{}" --once
"""
def main():
    while not rospy.is_shutdown():
        print "start while-----"
        t=0
        # vel=0.2
        # ace=50
        vel=1.05
        ace=1.4
        qq=[[-45, -180, 90, -180, -90, 180]
            # [-46.6821800834981, -174.97864208684973, 80.07423636487007, -186.04617133491166, -90.16842631753586, 182.8492880891567]
            # [-46.705208749303715, -175.06754382908662, 80.11438968096274, -186.14081863227838, -90.21481969445425, -0.6345364125868596]
        #[#-46.72747516599795, -175.15460415232326, 80.15343104225853, -186.23553423153677, -90.25757394658287, -24.25125691705894]
#82792, -95.66279987458553, -74.08597000725112, -10.44827788754191, 83.8387272278117, 0.47940733329694263]#20190220飞接触
            ]
        qzero = [-0.105882994329, -2.98965641657, 1.16991114616, -2.8067398707, -1.64681274096, 2.48291316032]

        for ii in xrange(len(qq)):
            qt=change_angle_to_pi(qq[ii])
            # qt=[-3.59860155164,-1.82648800883,-1.41735542252,0.0812199084238,1.27000134315,0.734254316924]
            # qt=[-3.66249992472,-1.27642603705,-1.9559700595,0.0701396996895,1.3338858418,0.73287290524]
            # time.sleep(1)
            # qt=[3.1897695779800417, -2.471768395100729, 1.8933079242706299, -2.4048668066607877, -3.1475941101657314, -3.8097620646106165]
            moveur(pub, qt,ace,vel,t)
            print "start "+str(ii)+" ur position-----"
            time.sleep(3)
        print "after while------"
        rate.sleep()
        # rospy.spin()
if __name__=="__main__":
    main()