from numpy import *
import math
#####ur5 robot class #####
class aubo5_robot:
    # basic parameters
    NAME = 'robot'
    JOINT_SIZE = 0
    A = []
    ALPHA = []
    D = []
    JOINT_TYPE = []
    THETA = []
    # other parameters

    def __init__(self, name, joint_size, a, alpha, d, theta):
        self.NAME = name
        self.JOINT_SIZE = joint_size
        self.A = a
        self.ALPHA = alpha
        self.D = d
        self.THETA = theta
        # joint_type
        return;

    # modify DH method (Creig`s book)
    def MDH(self, a, alpha, d, cta):
        ans = array([[cos(cta),            -sin(cta),             0,           a],
                     [sin(cta)*cos(alpha),  cos(cta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [sin(cta)*sin(alpha),  cos(cta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                     [0,                    0,                    0,           1]])
        return ans;

    def fk(self, theta):
        T = array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        for k in range(0, self.JOINT_SIZE):
            T = dot(T, self.MDH(self.A[k], self.ALPHA[k], self.D[k], theta[k]))
        return T

    def dk(self, theta):
        T = zeros((self.JOINT_SIZE+1, 4, 4));
        T[0] = array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        for k in range(0, self.JOINT_SIZE):
            T[k+1] = dot(T[k], self.MDH(self.A[k], self.ALPHA[k], self.D[k], theta[k]))
        J = zeros((6, 6))
        for k in range(0, self.JOINT_SIZE):
            J[0:3, k] = cross(T[k+1][0:3, 2], T[self.JOINT_SIZE][0:3, 3] -T[k+1][0:3, 3]).transpose()
            # print J[0:3,k]
            J[3:6, k] = T[k+1][0:3, 2]
        # print "j",J[:2,...]
        J[:2,...]=-1*J[:2,...]
        J[3:5,...]=-1*J[3:5,...]
        return J

    def sk(self, w):
        return array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

    def ik(self, Rt, Pt, q):
        q_ans = q.copy()
        Tc = self.fk(q[0:6])
        error = Pt - Tc[0:3, 3]
        count = 0
        while linalg.norm(error) > 1e-5 and count < 10:
            J = self.dk(q[0:6])
            if fabs(linalg.det(J)) < 1e-5:
                print('singularity')
                return q_ans
            dv = error
            dw = 0.5*(cross(Tc[0:3, 0], Rt[0:3, 0]) + cross(Tc[0:3, 1], Rt[0:3, 1]) + cross(Tc[0:3, 2], Rt[0:3, 2]))
            #L = -0.5*(dot(self.sk(Rt[0:3, 0]), self.sk(Tc[0:3, 0])) + dot(self.sk(Rt[0:3, 1]), self.sk(Tc[0:3, 1])) + dot(self.sk(Rt[0:3, 2]), self.sk(Tc[0:3, 2])))
            #dw = dot(linalg.pinv(L), dw)
            #Re = dot(mat(Tc[0:3, 0:3]).T, Rt)
            #e  = array([0.5*(Re[2, 1] - Re[1, 2]), 0.5*(Re[0, 2] - Re[2, 0]), 0.5*(Re[1, 0] - Re[0, 1])])
            #dw = dot(Tc[0:3, 0:3], e)
            dx = array([dv[0], dv[1], dv[2], dw[0], dw[1], dw[2]])*0.5
            dq = dot(linalg.pinv(J), dx)
            q[0:6] = q[0:6] + dq.flatten()

            Tc = self.fk(q[0:6])
            error = Pt - Tc[0:3, 3]
            count = count + 1
        print('iterates ', count, 'times')
        if count >= 10:
            print('iterates more than 10 times')
            return q_ans
        return q

def main():
    a=[0,0,0.408,0.376,0,0]
    #a=[0,0,-0.42500,-0.39225,0,0]#a0,a1,a2,a3,a4,a5
    alpha=[0,math.pi/2,0,0,math.pi/2,-math.pi/2]
    #d=[0.089159,0,0,0.10915,0.09465,0.0823]#ur5
    d=[0.0985,0.1215,0,0,0.1025,0.094]#d1,d2,d3,d4,d5,d6
    # a=[0,-0.42500,-0.39225,0,0,0]
    # alpha=[math.pi/2,0,0,math.pi/2,-math.pi/2,0]
    # d=[0.089159,0,0,0.10915,0.09465,0.0823]
    # q=[-1.4827777777777778, -3.14, 1.57, -3.14, -1.57, 3.14]
    qq=[-0.785, -3.14, 1.57, -3.14, -1.57, 3.14]
    # q=[
    #     -85, -180, 90, -180, -90, 180
    # ]
    q=qq
    ur=aubo5_robot("ur5",6,a,alpha,d,q)
    
    print(ur.dk(q))
    # print ur.J('EE',q)
if __name__== '__main__':
    main()