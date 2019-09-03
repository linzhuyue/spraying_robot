#!/usr/bin/env python 
# -*- coding: utf-8 -*-
from math import *
import numpy
"""
a2 = 0.266;
a3 = 0.2565;
d1 = 0.157;
d2 = 0.119;
d5 = 0.1025;
d6 = 0.094;
#endif

#ifdef AUBO_I3s_PARAMS
a2 = 0.229;
a3 = 0.2015;
d1 = 0.1395;
d2 = 0.11055;
d5 = 0.08955;
d6 = 0.09055;
#endif

#define AUBO_I5_PARAMS
#ifdef AUBO_I5_PARAMS
a2 =  0.408;
a3 =  0.376;
d1 =  0.122;
d2 =  0.1215;
d5 =  0.1025;
d6 =  0.094;
#endif

#ifdef AUBO_I5s_PARAMS
a2 = 0.245;
a3 = 0.215;
d1 = 0.1215;
d2 = 0.1215;
d5 = 0.1025;
d6 = 0.094;
#endif

#ifdef AUBO_I5l_PARAMS
a2 = 0.608;
a3 = 0.6395;
d1 = 0.1215;
d2 = 0.1215;
d5 = 0.1025;
d6 = 0.094;
#endif

#ifdef AUBO_I7_PARAMS
a2 = 0.552;
a3 = 0.495;
d1 = 0.1632;
d2 = 0.178;
d5 = 0.1025;
d6 = 0.094;
#endif


#ifdef AUBO_I10_PARAMS
a2 = 0.647;
a3 = 0.6005;
d1 = 0.1632;
d2 = 0.2013;
d5 = 0.1025;
d6 = 0.094;
"""
class Aubo_kinematics():
    def __init__(self):
        self.a2 =  0.408
        self.a3 =  0.376
        self.d1 =  0.122
        self.d2 =  0.1215
        self.d5 =  0.1025
        self.d6 =  0.094
        self.ZERO_THRESH = 1e-4
        self.ARM_DOF=6
    def degree_to_rad(self,q):
        temp=[]
        for i in range(len(q)):
            temp.append(q[i]*pi/180)
        return temp
    def antiSinCos(self,sA,cA):
    
        eps = 1e-8
        angle = 0
        if((abs(sA) < eps)and(abs(cA) < eps)):
        
            return 0
        
        if(abs(cA) < eps):
            angle = pi/2.0*self.SIGN(sA)
        elif(abs(sA) < eps):
        
            if (self.SIGN(cA) == 1):
                angle = 0
            else:
                angle = pi
        
        else:
        
            angle = atan2(sA, cA)
        

        return angle
    def SIGN(self,x):
        return (x > 0) - (x < 0)
    
    def aubo_forward(self,q):
        q=self.degree_to_rad(q)
        T=[]
        for i in range(16):
            T.append(0)
        # print q
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        q4 = q[3]
        q5 = q[4]
        q6 = q[5]
        C1 = cos(q1)
        C2 = cos(q2)
        C4 = cos(q4)
        C5 = cos(q5)
        C6 = cos(q6)
        C23 = cos(q2 - q3)
        C234 = cos(q2 - q3 + q4)
        C2345 = cos(q2 - q3 + q4 - q5)
        C2345p = cos(q2 - q3 + q4 + q5)
        S1 = sin(q1)
        S2 = sin(q2)
        S4 = sin(q4)
        S5 = sin(q5)
        S6 = sin(q6)
        S23 = sin(q2 - q3)
        S234 = sin(q2 - q3 + q4)

        T[0] = -C6 * S1 * S5 + C1 * (C234 * C5 * C6 - S234 * S6)
        T[1]= S1 * S5 * S6 - C1 * (C4 * C6 * S23 + C23 * C6 * S4 + C234 * C5 * S6)
        T[2] = C5 * S1 + C1 * C234 * S5
        T[3] = (self.d2 + C5 * self.d6) * S1 - C1 * (self.a2 * S2 + (self.a3 + C4 * self.d5) * S23 + C23 * self.d5 * S4 - C234 * self.d6 * S5)

        T[4] = C234 * C5 * C6 * S1 + C1 * C6 * S5 - S1 * S234 * S6
        T[5] = -C6 * S1 * S234 - (C234 * C5 * S1 + C1 * S5) * S6
        T[6] = -C1 * C5 + C234 * S1 * S5
        T[7] = -C1 * (self.d2 + C5 * self.d6) - S1 * (self.a2 * S2 + (self.a3 + C4 * self.d5) * S23 + C23 * self.d5 * S4 - C234 * self.d6 * S5)

        T[8] = C5 * C6 * S234 + C234 * S6
        T[9] = C234 * C6 - C5 * S234 * S6
        T[10] = S234 * S5
        T[11] = self.d1 + self.a2 * C2 + self.a3 * C23 + self.d5 * C234 + self.d6 * C2345/2 - self.d6 * C2345p / 2
        T[12]=0
        T[13]=0
        T[14]=0
        T[15]=1
        return T
    def aubo_inverse(self,T):
        q_reslut_dic={}
        q_reslut=[]
        singularity = False

        num_sols = 0
        nx = T[0]
        ox = T[1]
        ax = T[2]
        px = T[3]

        ny = T[4]
        oy = T[5]
        ay = T[6] 
        py = T[7]
        nz = T[8]
        oz = T[9] 
        az = T[10] 
        pz = T[11]

        # //////////////////////// shoulder rotate joint (q1) //////////////////////////////
        q1=[0,0]

        A1 = self.d6 * ay - py
        B1 = self.d6 * ax - px
        R1 = A1 * A1 + B1 * B1 - self.d2 * self.d2


        if R1 < 0.0:
            return num_sols
        else:
            R12 = sqrt(R1)
            q1[0] =  self.antiSinCos(A1, B1) -  self.antiSinCos(self.d2, R12)
            q1[1] =  self.antiSinCos(A1, B1) -  self.antiSinCos(self.d2, -R12)
            for i in range(len(q1)):
            
                while q1[i] > pi:
                    q1[i] -= 2 * pi
                while q1[i] < -pi:
                    q1[i] += 2 * pi
            
        

        #////////////////////////////// wrist 2 joint (q5) //////////////////////////////
        q5=[[0,0],[0,0]]

        for i in range(len(q5)):
        

            C1 = cos(q1[i])
            S1 = sin(q1[i])
            B5 = -ay * C1 + ax * S1
            M5 = (-ny * C1 + nx * S1)
            N5 = (-oy * C1 + ox * S1)
            R5 = sqrt(M5 * M5 + N5 * N5)

            q5[i][0] = self.antiSinCos(R5, B5)
            q5[i][1] = self.antiSinCos(-R5, B5)
        

        #////////////////////////////////////////////////////////////////////////////////

        #////////////////////////////// wrist 3 joint (q6) //////////////////////////////
        q6=0

        q3=[0,0]
        q2=[0,0]
        q4=[0,0]
        for i in range(len(q3)):
        
            for j in range(len(q3)):
            
                #// wrist 3 joint (q6) //
                C1 = cos(q1[i])
                S1 = sin(q1[i])
                S5 = sin(q5[i][j])

                A6 = (-oy * C1 + ox * S1)
                B6 = (ny * C1 - nx * S1)

                if fabs(S5) < self.ZERO_THRESH:# //the condition is only dependent on q1
                
                    singularity = True
                    break
                else:
                    q6 = self.antiSinCos(A6 * S5, B6 * S5)

                #/////// joints (q3,q2,q4) //////
                C6 = cos(q6)
                S6 = sin(q6)

                pp1 = C1 * (ax * self.d6 - px + self.d5 * ox * C6 + self.d5 * nx * S6) + S1 * (ay * self.d6 - py + self.d5 * oy * C6 + self.d5 * ny * S6)
                pp2 = -self.d1 - az * self.d6 + pz - self.d5 * oz * C6 - self.d5 * nz * S6
                B3 = (pp1 * pp1 + pp2 * pp2 - self.a2 * self.a2 - self.a3 * self.a3) / (2 * self.a2 * self.a3)


                if((1 - B3 * B3) < self.ZERO_THRESH):
                    singularity = True
                    continue
                else:
                    Sin3 = sqrt(1 - B3 * B3)
                    q3[0] = self.antiSinCos(Sin3, B3)
                    q3[1] = self.antiSinCos(-Sin3, B3)

                for k in range(len(q3)):
                

                    C3 = cos(q3[k])
                    S3 = sin(q3[k])
                    A2 = pp1 * (self.a2 + self.a3 * C3) + pp2 * (self.a3 * S3)
                    B2 = pp2 * (self.a2 + self.a3 * C3) - pp1 * (self.a3 * S3)

                    q2[k] = self.antiSinCos(A2, B2)
                    C2 = cos(q2[k])
                    S2 = sin(q2[k])

                    A4 = -C1 * (ox * C6 + nx * S6) - S1 * (oy * C6 + ny * S6)
                    B4 = oz * C6 + nz * S6
                    A41 = pp1 - self.a2 * S2
                    B41 = pp2 - self.a2 * C2

                    q4[k] = self.antiSinCos(A4, B4) - self.antiSinCos(A41, B41)
                    while(q4[k] > pi):
                        q4[k] -= 2 * pi
                    while(q4[k] < -pi):
                        q4[k] += 2 * pi
                    q_reslut=[q1[i],q2[k],q3[k],q4[k],q5[i][j],q6]

                    q_reslut_dic.update({num_sols:q_reslut})
                    num_sols+=1
                
        return q_reslut_dic,num_sols
        """
        The Frobenius norm, sometimes also called the Euclidean norm (a term unfortunately also used for the vector L^2-norm), 
        is matrix norm of an m×n matrix A defined as the square root of the sum of the absolute squares of its elements,
        """
    def List_Frobenius_Norm(self,list_a,list_b):
        new_list=[]
        if len(list_a)==len(list_b):
            for i in range(len(list_a)):
                new_list.append(abs(list_a[i]-list_b[i])**2)
        else:
            print("please make sure the list has the same length")
        
        return sqrt(self.sum_list(new_list))
    def sum_list(self,list_data):
        sum_data=0
        for i in range(len(list_data)):
            sum_data+=list_data[i]
        return sum_data
    # //choose solution ;input all q_sols,and last q_old;
    # //out put the nearest q solution;
    # /**
    #  * @brief chooseIKonRefJoint , choose mode == 0;
    #  * @param q_sols rad
    #  * @param q_ref
    #  * @param q_choose
    #  * @return
    #  */
    def chooseIKonRefJoint(self,q_sols,q_ref):
    
        nn = len(q_sols)
        if(nn == 0):
            return False,[]
        # print "nn---",nn
        sum_data = self.List_Frobenius_Norm(q_ref,q_sols[0])
        err=0
        index = 0
        for i in range(len(q_sols)):
        
            err = self.List_Frobenius_Norm(q_ref,q_sols[i])
            # print "err",err
            if(err < sum_data):
                index = i
                sum_data = err
            
        # print sum_data
        q_choose = q_sols[index]

        return True,q_choose
    """aubo rotation joint:-175 degree/175 degree,here is rad I wanna take a break so use degree"""
    def selectIK(self,q_sols,AngleLimit):
        q_sols_selected={}
        N = len(q_sols)
        # print "selectIK ---N---",N
        if( N == 0):
            return False,{}
        num = 0
        valid = True
        for i in range(N):
        
            valid = True;
            for j in range(self.ARM_DOF):
            #drop greater than offical degree
                if(q_sols[i][j] > AngleLimit[j][1] or q_sols[i][j] < AngleLimit[j][0]):
                    valid = False
                    break
                
            #delete the sols about joint angular increase 2*pi greater than the legal angular
            if(valid):
                temp=q_sols[i]
                temp_1=q_sols[i]
                for j in range(self.ARM_DOF):
                    temp[j] += 2*pi
                    temp_1[j]-=2*pi
                    if temp[j]>AngleLimit[j][1] or temp_1[j]<AngleLimit[j][0]:
                        valid = False
                        break
                if valid:
                    q_sols_selected.update({num:q_sols[i]})
                    num+=1

        num_sols = num;

        if(num > 0):
            return True,q_sols_selected
        else:
            return False,{}
    

    def GetInverseResult(self,T_target,q_ref):
    
        num_sols = 0

        maxq = 175.0/180.0*pi
        AngleLimit = [(-maxq,maxq),(-maxq,maxq),(-maxq,maxq),(-maxq,maxq),(-maxq,maxq),(-maxq,maxq)]

        #inverse and remove zero list
        q_sols_all,num_sols = self.aubo_inverse(T_target)
        
        if(len(q_sols_all) != 0):
            for i in q_sols_all:
                print("num:"+str(i)+' '+"sols",q_sols_all[i])
            #remove not in limited data 
            ret2,q_sols_inlimit = self.selectIK(q_sols_all, AngleLimit)
            # print "q_sols_inlimit",q_sols_inlimit
            if((len(q_sols_inlimit) != 0) and (True == ret2)):
            
                ret3,q_result = self.chooseIKonRefJoint(q_sols_inlimit, q_ref)

                if(True == ret3):
                
                    print(" find solution choose  ")
                    return q_result
                
                else:
                
                    print(" No solution choose  ")
            else:
            
                print("no valid sols ")

            
        
        else:
        
            print("inverse result num is 0")
            # return False
        
    
def main():
    ak47=Aubo_kinematics()
    # print ak47.aubo_forward([-3.3364,12.406,-81.09,-91.207,-86.08,0.164])
    # print numpy.matrix(ak47.aubo_forward([-3.3364,12.406,-81.09,-91.207,-86.08,0.164])).reshape((4,4))
    tt=[0.010016939985065143, -0.039901099098502056, -0.9991534232559417, -0.3, -0.999934201568705, 0.005186605233011846, -0.010231894219208601, -0.09507448660946277, 0.005590478198847001, 0.999190172798396, -0.039846519755429126, 0.5962177031402299, 0, 0, 0, 1]
    # tt=[1.0, 0.0, 0.0, -0.4, 0.0, -1.0, -0.0, -0.8500000000000001, 0.0, 0.0, -1.0, -0.4, 0.0, 0.0, 0.0, 1.0]
    # tt=[1.0, 0.0, 0.0, -0.4, 0.0, -1.0, -0.0, -0.4500000000000001, 0.0, 0.0, -1.0, -0.4, 0.0, 0.0, 0.0, 1.0]
    # q_dict,num=ak47.aubo_inverse(tt)
    # print q_dict,num
    # for i in range(len(q_dict)):
    #     print i,q_dict[i]
    # print ak47.degree_to_rad([-3.3364,12.406,-81.09,-91.207,-86.08,0.164])
    print ak47.GetInverseResult(tt,ak47.degree_to_rad([-3.3364,12.406,-81.09,-91.207,-86.08,0.164]))
if __name__=="__main__":
    main()