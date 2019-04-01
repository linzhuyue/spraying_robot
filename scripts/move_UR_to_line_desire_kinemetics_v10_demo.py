#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy
from std_msgs.msg import String,Float64
from frompitoangle import *
from ur5_kinematics import *
from ur5_pose_get import *
from transfer import *
from trans_methods import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from ur_tool_velocity_sub import *
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import serial
import time
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


class UrLineCircle:
    def __init__(self,weights,radius,urdfname,p,Kp):
        self.weights = weights
        self.radius=radius#m
        self.vel = 0.1
        self.ace = 50
        self.cont=50
        self.t=0
        self.theta=-math.pi / 4
        self.tempq=[]
        self.p=p
        self.Kp=Kp
        self.urdfname=urdfname
        # rotating 45 degree with Z aisx
        self.wRb = [math.cos(self.theta), -1*math.sin(self.theta), 0, math.sin(self.theta), math.cos(self.theta), 0, 0, 0,1]

        self.uree_velocity_q1_pub = rospy.Publisher("/uree_velocity_q1", Float64, queue_size=10)
        self.uree_velocity_q2_pub = rospy.Publisher("uree_velocity_q2", Float64, queue_size=10)
        self.uree_velocity_q3_pub = rospy.Publisher("uree_velocity_q3", Float64, queue_size=10)
        self.uree_velocity_q4_pub = rospy.Publisher("uree_velocity_q4", Float64, queue_size=10)
        self.uree_velocity_q5_pub = rospy.Publisher("uree_velocity_q5", Float64, queue_size=10)
        self.uree_velocity_q6_pub = rospy.Publisher("uree_velocity_q6", Float64, queue_size=10)

        self.uree_world_v_x_pub = rospy.Publisher("/ur_world_frame_ee_v_x", Float64, queue_size=10)
        self.uree_world_v_y_pub = rospy.Publisher("/ur_world_frame_ee_v_y", Float64, queue_size=10)
        self.uree_world_v_z_pub = rospy.Publisher("/ur_world_frame_ee_v_z", Float64, queue_size=10)
        self.jacabian_det_pub = rospy.Publisher("/ur_jacabian_det", Float64, queue_size=10)
        self.jacabian_rank_pub = rospy.Publisher("/ur_jacabian_rank", Float64, queue_size=10)
    def Init_node(self):
        rospy.init_node("move_ur5_circle")
        pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        return pub
    def get_urobject_ur5kinetmatics(self):
        ur0 = Kinematic()
        return ur0
    def get_jacabian_from_joint(self,jointq):
        #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        robot = URDF.from_xml_file(self.urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        # print tree.getNrOfSegments()
        chain = tree.getChain("base_link", "ee_link")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        q=jointq
        #q = [0, 0, 1, 0, 1, 0]
        pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        # print pose
        #print list(pose)
        q0=Kinematic()
        # if flag==1:
        #     q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward(q))
        # else:
        q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # print "----------iverse-------------------\n", q_ik

        if q_ik is not None:
            pose_sol = kdl_kin.forward(q_ik)  # should equal pose
            print "------------------forward ------------------\n",pose_sol

        J = kdl_kin.jacobian(q)
        #print 'J:', J
        return J,pose

    def caculate_vlocity_by_jocabian_impedance(self,t,Tstar,q_joint_t):
        """

        :return:
        """
        Jacabian_t,pose=self.get_jacabian_from_joint(q_joint_t)
        Jacabian_singularity=numpy.matrix(Jacabian_t).reshape(6,6)+self.p*numpy.eye(6)
        print "Jacabian_t",Jacabian_singularity
        print "tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T",tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T
        qdot_t=numpy.dot(Jacabian_singularity.I,tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T)
        print "qdot_t",self.Kp*t*qdot_t.T
        print "numpy.array(q_joint_t).T",numpy.array(q_joint_t).T
        # jacabian_rank=
        new_q_t_1=numpy.array(q_joint_t).T+self.Kp*t*qdot_t.T

        print "new_q_t1",new_q_t_1
        return new_q_t_1
    def caculate_vlocity_by_jocabian_pinv(self,t,Tstar,q_joint_t):
        """

        :return:
        """
        Jacabian_t,pose=self.get_jacabian_from_joint(q_joint_t)
        Jacabian_plus=numpy.dot(numpy.dot(Jacabian_t.T,Jacabian_t).I,Jacabian_t.T)
        # Jacabian_singularity=numpy.matrix(Jacabian_t).reshape(6,6)+self.p*numpy.eye(6)
        print "Jacabian_plus",Jacabian_plus.reshape(6,6)
        print "tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T",tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T
        qdot_t=numpy.dot(Jacabian_plus.I,tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T)
        print "qdot_t",self.Kp*t*qdot_t.T
        print "numpy.array(q_joint_t).T",numpy.array(q_joint_t).T
        new_q_t_1=numpy.array(q_joint_t).T+self.Kp*1*qdot_t.T

        print "new_q_t1",new_q_t_1
        return new_q_t_1

    def caculate_vlocity_by_jocabian_v1(self, cn, q_joint_t, deltax, flagx, flagy):

        Jacabian_t, pose = self.get_jacabian_from_joint(q_joint_t)
        # Jacabian_plus=numpy.dot(numpy.dot(Jacabian_t.T,Jacabian_t).I,Jacabian_t.T)
        # # Jacabian_singularity=numpy.matrix(Jacabian_t).reshape(6,6)+self.p*numpy.eye(6)
        # print "real time pose x",(pose[0].tolist()[0][3])
        # print "Tstar",Tstar[3],Tstar[3]-(pose[0].tolist()[0][3])
        # print "Jacabian_plus",Jacabian_plus.reshape(6,6)
        # print "tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T",tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T
        Jacabian_plus = numpy.dot(numpy.dot(Jacabian_t.T, Jacabian_t).I, Jacabian_t.T)
        new_deta_v = numpy.array([[flagx * deltax, flagy * deltax, 0, 0, 0]])



        new_jacabian_t = Jacabian_t[:5, :5]
        jacabian_det = numpy.linalg.det(new_jacabian_t)
        qdot_t = numpy.dot(new_jacabian_t.I, new_deta_v.T).tolist()
        new_qdot_t = numpy.array([[qdot_t[0][0]], [qdot_t[1][0]], [qdot_t[2][0]], [qdot_t[3][0]], [qdot_t[4][0]], [0]])
        print "jacabian_det", type(jacabian_det), jacabian_det
        # qdot_t = numpy.dot(Jacabian_t.I, new_deta_v.T)
        qdot_dot = new_qdot_t.tolist()
        print "qdot_dot", qdot_dot
        self.uree_velocity_q1_pub.publish(qdot_dot[0][0])
        self.uree_velocity_q2_pub.publish(qdot_dot[1][0])
        self.uree_velocity_q3_pub.publish(qdot_dot[2][0])
        self.uree_velocity_q4_pub.publish(qdot_dot[3][0])
        self.uree_velocity_q5_pub.publish(qdot_dot[4][0])
        self.uree_velocity_q6_pub.publish(qdot_dot[5][0])
        if jacabian_det == 0:
            print "jacabian_det-------is zero-------"
        # self.jacabian_rank_pub.publish()
        # print "qdot_t",self.Kp*1*qdot_t.T
        # print "numpy.array(q_joint_t).T",numpy.array(q_joint_t).T
        jacabian_rank = numpy.linalg.matrix_rank(new_jacabian_t)

        self.jacabian_rank_pub.publish(jacabian_rank)
        self.jacabian_det_pub.publish(jacabian_det)

        new_q_t_1 = numpy.array(q_joint_t).T + self.Kp * cn * new_qdot_t.T

        print "new_q_t1", new_q_t_1
        return new_q_t_1
    def caculate_vlocity_by_jocabian(self,cn,q_joint_t,deltax,flagx,flagy):

        Jacabian_t,pose=self.get_jacabian_from_joint(q_joint_t)
        # Jacabian_plus=numpy.dot(numpy.dot(Jacabian_t.T,Jacabian_t).I,Jacabian_t.T)
        # # Jacabian_singularity=numpy.matrix(Jacabian_t).reshape(6,6)+self.p*numpy.eye(6)
        # print "real time pose x",(pose[0].tolist()[0][3])
        # print "Tstar",Tstar[3],Tstar[3]-(pose[0].tolist()[0][3])
        # print "Jacabian_plus",Jacabian_plus.reshape(6,6)
        # print "tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T",tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T
        Jacabian_plus = numpy.dot(numpy.dot(Jacabian_t.T, Jacabian_t).I, Jacabian_t.T)
        new_deta_v=numpy.array([[flagx*deltax,flagy*deltax,0,0,0,0]])

        jacabian_det=numpy.linalg.det(Jacabian_t)

        print "jacabian_det",type(jacabian_det),jacabian_det
        qdot_t=numpy.dot(Jacabian_t.I,new_deta_v.T)
        qdot_dot=qdot_t.tolist()
        print "qdot_dot",qdot_dot
        self.uree_velocity_q1_pub.publish(qdot_dot[0][0])
        self.uree_velocity_q2_pub.publish(qdot_dot[1][0])
        self.uree_velocity_q3_pub.publish(qdot_dot[2][0])
        self.uree_velocity_q4_pub.publish(qdot_dot[3][0])
        self.uree_velocity_q5_pub.publish(qdot_dot[4][0])
        self.uree_velocity_q6_pub.publish(qdot_dot[5][0])
        if jacabian_det==0:
            print "jacabian_det-------is zero-------"
        # self.jacabian_rank_pub.publish()
        # print "qdot_t",self.Kp*1*qdot_t.T
        # print "numpy.array(q_joint_t).T",numpy.array(q_joint_t).T
        jacabian_rank=numpy.linalg.matrix_rank(Jacabian_t)

        self.jacabian_rank_pub.publish(jacabian_rank)
        self.jacabian_det_pub.publish(jacabian_det)

        new_q_t_1=numpy.array(q_joint_t).T+self.Kp*cn*qdot_t.T

        print "new_q_t1",new_q_t_1
        return new_q_t_1
    def get_draw_circle_xy(self,t,xy_center_pos):
        x = xy_center_pos[0] + self.radius * math.cos( 2 * math.pi * t / self.cont )
        y = xy_center_pos[1] + self.radius * math.sin( 2 * math.pi * t / self.cont)
        return  [x,y]
    def get_draw_line_x(self,transxyz0,transxyz_d):#transxyz[x,y,z]
        xn_1=1*(transxyz_d[0]-transxyz0[0])/self.cont
        return xn_1

    def get_IK_from_T(self,T,q_last):
        ur0 = self.get_urobject_ur5kinetmatics()
        return ur0.best_sol(self.weights,q_last,T)

    def get_T_translation(self, T):
        trans_x = T[3]
        trans_y = T[7]
        trans_z = T[11]
        return [trans_x, trans_y, trans_z]
    def insert_new_xy(self,T,nx,ny,nz):
        temp=[]
        for i in xrange(16):
            if i==3:
                temp.append(nx)
            elif i==7:
                temp.append(ny)
            elif i == 11:
                temp.append(nz)
            else:
                temp.append(T[i])
        return temp
    def numpyarray_tolist(self,T):
        tt=T.tolist()
        temp=[]
        for i in range(4):
            for j in range(4):
                temp.append(tt[i][j])
        return temp

    def urscript_pub(self, pub, qq, vel, ace, t):

        ss = "movej([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(
            qq[3]) + "," + str(qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
            vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
            # ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
        pub.publish(ss)

    """
        [RwbRbe Rwbtbe
        0          1  ]
        """

    def caculate_world_frame(self, T):
        bRe = tr2r(T)
        bte = transl(T)
        wRb = numpy.array(self.wRb).reshape((3, 3))
        homegeneous_T_part3 = numpy.array([0, 0, 0, 1])
        wRbbRe = numpy.dot(wRb, bRe)
        wRbbte = numpy.dot(wRb, bte)
        new_T_T = numpy.column_stack((wRbbRe, wRbbte))
        new_T = np.row_stack((new_T_T, homegeneous_T_part3))
        last_T = self.numpyarray_tolist(new_T)
        inv_wTb_part = numpy.array([0, 0, 0]).T
        inv_wTb_1 = numpy.column_stack((wRb, inv_wTb_part))
        inv_wTb_2 = np.row_stack((inv_wTb_1, homegeneous_T_part3))
        return last_T, inv_wTb_2

    def move_ee(self,ur5_pub,q_now_t,deltax,cn,flagx,flagy):
        q_new_from_jacabian=self.caculate_vlocity_by_jocabian(cn,q_now_t,deltax,flagx,flagy).tolist()[0]

        self.urscript_pub(ur5_pub, q_new_from_jacabian, self.vel, self.ace, self.t)
        return q_new_from_jacabian


def main():
    t=0
    vel=0.1
    ace=50
    # vel=1.05
    # ace=1.4

    urdfname = "/data/ros/ur_ws_yue/src/ur5_planning/urdf/ur5.urdf"
    qstart=[-45, -180, 90, -156, -90, 180]#[-45, -180, 90, -180, -90, 180]
        # [-46.658908262958036, -174.89198611081196, 80.03454750164096, -168.48910996772918, -90.12196190665009, 182.75583607489475]

    ratet = 1.5
    radius=0.1
    weights = [1.] * 6
    T_list=[]
    p=0.001
    Kp=1
    urc=UrLineCircle(weights,radius,urdfname,p,Kp)
    pub=urc.Init_node()
    rate = rospy.Rate(ratet)

    # first step go to initial pos
    qzero = display(getpi(qstart))
    # urc.urscript_pub(pub,q,vel,ace,t)
    # second get T use urkinematics
    urk = urc.get_urobject_ur5kinetmatics()
    F_T = urk.Forward(qzero)
    # ur5_pub=urc.
    # print "F_T", F_T
    TransT = urc.get_T_translation(F_T)

    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    tool0 = UrToolVelocityRead()
    tool_sub = rospy.Subscriber("/tool_velocity", TwistStamped, tool0.Ur_tool_velocity_callback)

    WTe, inv_wTb = urc.caculate_world_frame(numpy.array(F_T).reshape((4, 4)))
    ur0_kinematics=urc.get_urobject_ur5kinetmatics()
    cn=1


    flag_to_zero=0
    flag_to_right=0
    flag_to_down_1=0
    flag_to_down_2=0
    flag_to_down_3=0
    flag_to_down_4=0
    flag_to_down_5=0
    flag_to_down_6 = 0
    flag_to_left_1=0
    flag_to_left_2=0
    flag_to_left_3=0
    flag_to_left_4=0
    flag_to_left_5=0
    flag_to_right_1=0
    flag_to_right_2 = 0
    flag_to_right_3=0
    flag_to_right_4=0
    flag_to_right_5=0
    go_back_start_flag=0
    while not rospy.is_shutdown():

        if len(ur_reader.ave_ur_pose)!=0:
            q_now = ur_reader.ave_ur_pose
            """
                    go to the largest distance 
                    """
            deltax = urc.get_draw_line_x([0.286, 0, 0], [0.5, 0, 0])
            if flag_to_zero == 0:
                print cn, "go to the largest distance  -----", q_now
                urc.move_ee(pub,q_now,deltax,cn,1,1)
                cn += 1
                # time.sleep(0.1)
                if cn == urc.cont - 10:
                    flag_to_zero = 1
                    flag_to_right = 1
                    cn = 1
            if flag_to_right == 1:
                print "first move to right -----"
                # deltax = urc.get_draw_line_x([0.286, 0, 0], [0.5, 0, 0])
                qq = urc.move_ee(pub,q_now,deltax,cn,-1,-1)
                print cn, "move to right -----", qq
                cn += 1
                if cn == int((urc.cont - 10)*3/2):
                    flag_to_down_1 = 1
                    flag_to_right = 0
                    # time.sleep(1)
                    cn = 1
            if flag_to_down_1 == 1:
                print "first move to down -----"
                # detay = urc.get_draw_line_x(cn, [-0.45, 0, 0], [0.45, 0, 0])
                qq = urc.move_ee(pub,q_now,deltax,cn,1,-1)
                print cn, "first move to down -----", qq
                cn += 1
                if cn == int((urc.cont)/3):
                    flag_to_down_1 = 0
                    flag_to_left_1 = 1
                    cn = 1
            if flag_to_left_1 == 1:
                print "first move to left -----"
                qq = urc.move_ee(pub,q_now,deltax,cn,1,1)
                print cn, "first move to left -----", qq
                cn += 1

                if cn == int((urc.cont - 10)*3/2):
                    flag_to_left_1 = 0
                    flag_to_down_2 = 1
                    # go_back_start_flag=1
                    cn = 1
            if flag_to_down_2 == 1:
                print "second move to down -----"

                qq = urc.move_ee(pub,q_now,deltax,cn,1,-1)
                print cn, "second move to  down -----", qq
                cn += 1
                if cn == int((urc.cont)/3):
                    flag_to_down_2 = 0
                    flag_to_right_2 = 1
                    # time.sleep(1.5)
                    cn = 1
            if flag_to_right_2 == 1:
                print "second move to right -----"

                qq = urc.move_ee(pub, q_now, deltax, cn, -1, -1)
                print cn, "second move to right -----", qq
                cn += 1

                if cn == int((urc.cont - 10)*3/2):

                    flag_to_right_2 = 0
                    flag_to_down_3 = 1

                    cn = 1
            if flag_to_down_3 == 1:
                print "third move to down -----"
                qq = urc.move_ee(pub, q_now, deltax, cn, 1, -1)
                print cn, "third move to down -----", qq
                cn += 1
                if cn == int((urc.cont)/10):
                    flag_to_down_3 = 0
                    flag_to_left_2 = 1
                    cn = 1
            if flag_to_left_2 == 1:
                print "third move to left -----"

                qq = urc.move_ee(pub, q_now, deltax, cn, 1, 1)
                print cn, "third move to left -----", qq
                cn += 1
                if cn == int((urc.cont - 10)*3/2):

                    flag_to_left_2 = 0
                    flag_to_down_4 = 1
                    cn = 1
            if flag_to_down_4 == 1:
                print "fourth move to down -----"

                qq = urc.move_ee(pub, q_now, deltax, cn, 1, -1)

                print cn, "fourth move to down -----", qq
                cn += 1

                if cn == int((urc.cont)/10):

                    flag_to_down_4 = 0
                    flag_to_right_3 = 1

                    cn = 1
            if flag_to_right_3 == 1:
                print "fourth move to right -----"

                qq = urc.move_ee(pub, q_now, deltax, cn, -1, -1)
                print cn, "fourth move to right -----", qq
                cn += 1

                if cn == (urc.cont - 2):

                    flag_to_right_3 = 0
                    flag_to_down_5 = 1
                    time.sleep(1.5)
                    cn = 1
            if flag_to_down_5 == 1:
                print "fifth move to down -----"

                qq = urc.move_ee(pub, q_now, deltax, cn, 1, -1)
                print cn, "fifth move to down -----", qq
                cn += 1

                if cn == int((urc.cont)/10):

                    flag_to_down_5 = 0
                    flag_to_left_3 = 1

                    cn = 1
            if flag_to_left_3 == 1:
                print "fifth move to left -----"

                qq = urc.move_ee(pub, q_now, deltax, cn, 1, 1)
                print cn, "fifth move to left -----", qq
                cn += 1

                if cn == int((urc.cont - 10)*3/2):

                    flag_to_left_3 = 0
                    flag_to_down_6 = 1
                    cn = 1
            if flag_to_down_6==1:
                print "sixth move to down -----"

                qq = urc.move_ee(pub, q_now, deltax, cn, 1, -1)
                print cn, "sixth move to down -----", qq
                cn+=1

                if cn==int((urc.cont)/10):
                    flag_to_down_6=0
                    flag_to_right_4=1

                    cn=1
            if flag_to_right_4==1:
                print "sixth move to right -----"

                qq = urc.move_ee(pub, q_now, deltax, cn, -1, -1)
                print cn, "sixth move to right -----", qq
                cn+=1

                if cn==int((urc.cont - 10)*3/2):

                    flag_to_right_4=0
                    go_back_start_flag=1

                    cn=1
            if go_back_start_flag == 1:
                urc.urscript_pub(pub, qzero, 0.2, ace, t)
                time.sleep(3)
                cn = 1
                go_back_start_flag = 0
                print "path planning over ------"
            # rate.sleep()
            # if flag_to_right==1:
            #     print "first move to right -----"
            #     detax=urc.get_draw_line_x(cn,[0.286,0,0],[0.5,0,0])
            #     qq=urc.move_ee(pub,q_now,detax,cn,-1,-1)
            #     print cn, "move to right -----", qq
            #     cn+=1
            #     # time.sleep(0.1)
            #     q_now = ur_reader.ave_ur_pose
            #
            #     if cn==(urc.cont-10):
            #
            #         flag_to_zero=1
            #         flag_to_right=0
            #         time.sleep(4)
            #         cn=1

        rate.sleep()
if __name__ == '__main__':
        main()