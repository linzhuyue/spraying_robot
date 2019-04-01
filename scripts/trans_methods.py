#!usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
#from hand_in_eye import *
from Quaternion import *

def tr2jac_new(T,samebody):#eTc
    # T = np.array(T)
    R = tr2r(T)
    # jac = np.zeros((6, 6))
    """
    jac = [ jac_part1,  jac_part2;
            jac_part3,  jac_part4;
                ]
    """
    if samebody == 1:
        jac_part1 = R
        New_trans = np.dot(-1 * (R.T), transl(T))
        jac_part2 = -np.dot(R, skew(New_trans))
        print "self.transl(T))", transl(T)
        # T1=[1,2,3]
        print "self.skew(self.transl(T))\n", skew(New_trans)
        jac_part3 = np.zeros((3, 3))
        jac_part4 = R

    else:
        jac_part1 = R
        jac_part2 = np.zeros((3, 3))
        jac_part3 = np.zeros((3, 3))
        jac_part4 = R
    jac_row1 = np.column_stack((jac_part1, jac_part2))
    jac_row2 = np.column_stack((jac_part3, jac_part4))
    jac = np.row_stack((jac_row1, jac_row2))
    return jac
def tr2inver(T):
    R=tr2r(T)
    t=transl(T)
    tinverpart1=R.T
    tinverpart2=-1*np.dot(R.T,t)
    tinverpart3=np.array([0,0,0,1])
    tinvercolum=np.column_stack((tinverpart1,tinverpart2))
    print tinvercolum
    result=np.row_stack((tinvercolum,tinverpart3))
    return result
def tr2transports_1(T):
    R=tr2r(T)
    t=transl(T)
    # tinverpart1=R.T
    # rr = np.array([0, 0, 1, 0, 1, 0, -1, 0, 0]).reshape((3, 3))
    rr = np.array([0, 1,0, 0, 0, -1, -1, 0, 0]).reshape((3, 3))
    # rr=np.array([0,0,1,0,-1,0,1,0,0]).reshape((3,3))
    # rr = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1]).reshape((3, 3))
    tinverpart1=np.dot(rr,R.T)
    tinverpart2=np.dot(rr,-1*np.dot(R.T,t))
    tinverpart3=np.array([0,0,0,1])
    tinvercolum=np.column_stack((tinverpart1,tinverpart2))
    print tinvercolum
    result=np.row_stack((tinvercolum,tinverpart3))
    return result
def tr2transports(T):
    R=tr2r(T)
    t=transl(T)
    # tinverpart1=R.T
    rr = np.array([0, 0, 1, 0, -1, 0, 1, 0, 0]).reshape((3, 3))
    # rr=np.array([0,0,1,0,-1,0,1,0,0]).reshape((3,3))
    # rr = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1]).reshape((3, 3))
    tinverpart1=np.dot(R,rr).T
    tinverpart2=-1*np.dot(np.dot(R,rr).T,t)
    tinverpart3=np.array([0,0,0,1])
    tinvercolum=np.column_stack((tinverpart1,tinverpart2))
    print tinvercolum
    result=np.row_stack((tinvercolum,tinverpart3))
    return result
def tr2jac( T, samebody):
    #T = np.array(T)
    R = tr2r(T)
    #jac = np.zeros((6, 6))
    """
    jac = [ jac_part1,  jac_part2;
            jac_part3,  jac_part4;
                ]
    """
    if samebody==1:
        jac_part1 = R.T
        jac_part2 = -np.dot( R.T,skew( transl(T)))
        jac_part3 = np.zeros((3,3))
        jac_part4 = R.T

    else:
        jac_part1 = R.T
        jac_part2 = np.zeros((3,3))
        jac_part3 = np.zeros((3,3))
        jac_part4 = R.T
    jac_row1 = np.column_stack( (jac_part1, jac_part2) )
    jac_row2 = np.column_stack( (jac_part3, jac_part4) )
    jac = np.row_stack( (jac_row1, jac_row2) )
    return jac

"""
if l is 3*1 , then get 
skew(l) = [ 0, -l(2), l(1)
            l(2), 0 , -l(0)
            -l(1), l(0), 0]
if l is 1*1, then get
skew(l) = [ 0 , -l[0]
            l[0], 0 ]

"""
def skew(l):
    a, b = np.shape(l)
    try:
        if a == 3:
            s = np.array( [ 0, -l[2], l[1], l[2], 0, -l[0], -l[1], l[0], 0 ] )
            s = s.reshape((3,3))
            #print "s:", s
            return s
        elif a == 1:
            s = np.array( [ 0, -l[0], l[0], 0])
            s = s.reshape( (2,2) )
            return s
    except:
        print("erro l size!!!  3*1 or 1*1 required!")


def tr2r(T):
    r = [ 0, 1, 2]
    c = [ 0, 1, 2]
    R1 = T[r]
    R = R1[:,c]
    return R
def q2t(qutanion_info):
    trans = qutanion_info[:3]
    mtrans = np.matrix(trans)
    print "mtrans", mtrans.T
    addnum = [0, 0, 0, 1]
    addnum == np.matrix(addnum)
    print "addnum", addnum
    s = qutanion_info[6]
    v1 = qutanion_info[3]
    v2 = qutanion_info[4]
    v3 = qutanion_info[5]
    q = quaternion(s, v1, v2, v3)
    print "unit", q.unit()
    q2ro = q.r().T

    reslut=np.vstack((np.hstack((q2ro, mtrans.T)), addnum))
    return reslut
def transl(T):
    r = [3]
    c = [0 , 1, 2]
    l1 = T[:, r]
    l = l1[c]
    return l

def test_main():
    T=[-0.19476273940168198, -0.9808315529770444, -0.006077830648500385, -0.022917933508731628, -0.954529524844141, 0.1909585521035685, -0.22892841147251097, -0.07109668964896827, 0.2257008230857191, -0.038785255744265446, -0.973424286935189, 0.42859801647054574, 0.0, 0.0, 0.0, 1.0]
    T0=np.array(T).reshape((4,4))
    print tr2r(T0)
    print transl(T0)
    print np.dot(T0,tr2inver(T0))
    print T0
    # 1, get the X matrix
    # X = get_ur3_X()
    # print "rotation:", tr2r(X)
    # print "transition:", transl(X)
    # jac = tr2jac(X)
    # print "jac:", jac

    # a, b =np.shape(X[:,3])
    # print X[:,3]
    # print a,b

    # 2, get  the  samebody transfer matrix to jacobian matrix
    #jac = tr2jac(X)
    #print "jac", jac


if __name__=="__main__":
    test_main()