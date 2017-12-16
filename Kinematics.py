from __future__ import division
import numpy as np
from math import *
import math
from numpy.linalg import inv
from math import sin
from math import cos
from numpy import float64, deg2rad, dtype
import __main__
import unittest
from mpmath import jtheta, phi
from _socket import TCP_CORK

wcp=np.zeros((3,1))
wcp1=np.zeros((3,1))
poswcp=np.zeros((3,1))
Oriwcp=np.zeros((3,1))
A0_tcp=np.zeros((4,4),dtype=np.int)
A0_6=np.eye(4,4,dtype=np.float64)
FEUp=[0,0,0] #front elbow up
FEDown=[0,0,0] #front elbow down
REUp= [0,0,0] #front elbow up
REDown= [0,0,0] #front elbow down
DH_d=[815,0,0,1545,0,0,158]
DH_a=[350,1200,145,0,0,0,0]
Q3_REUp=0.0
Q3_REDown=0.0
Q2_REDown=0.0
Q2_REUp=0.0
Q3_FEUp=0.0
Q3_FEDown=0.0
Q2_FEDown=0.0
Q2_FEUp=0.0
Q1_F=0.0
Q1_R=0.0
class vectorspace(object):
    def __init__(self,theta_angles=[0,0,0,0,0,0]):
        self.theta_angles=theta_angles
        self.DH_d=[-815,0,0,-1545,0,0,-158]
        self.DH_a=[350,1200,145,0,0,0,0]
        self.DH_alpha=[math.radians(-90),0,math.radians(90),math.radians(90),math.radians(-90),0,0]
     
    def A01(self):
        return transformation(d=self.DH_d[0],a=self.DH_a[0],alpha=self.DH_alpha[0],theta=self.theta_angles[0])
 
    def A12(self):
        return transformation(d=self.DH_d[1],a=self.DH_a[1],alpha=self.DH_alpha[1],theta=self.theta_angles[1]+math.radians(-90))
 
    def A23(self):
        return transformation(d=self.DH_d[2],a=self.DH_a[2],alpha=self.DH_alpha[2],theta=self.theta_angles[2])
 
    def A34(self):
        return transformation(d=self.DH_d[3],a=self.DH_a[3],alpha=self.DH_alpha[3],theta=self.theta_angles[3])
 
    def A45(self):
        return transformation(d=self.DH_d[4],a=self.DH_a[4],alpha=self.DH_alpha[4],theta=self.theta_angles[4])
 
    def A56(self):
        return transformation(d=self.DH_d[5],a=self.DH_a[5],alpha=self.DH_alpha[5],theta=self.theta_angles[5])
     
    def A6tcp(self):
        return transformation(d=self.DH_d[6],a=self.DH_a[6],alpha=self.DH_alpha[6],theta=0)
    
    #Data sheet values
    def A3WRIST(self):
        return transformation(alpha=0, a=0, theta=0, d=self.DH_d[3])

    def base2tcp(self):
        return self.A12()*self.A23()*self.A34()*self.A45()*self.A56()*self.A6tcp()
    
    def base26(self):
        return self.A12()*self.A23()*self.A34()*self.A45()*self.A56()
    
    def baseToWrist(self):
        return self.A01() * self.A12() * self.A23() * self.A3WRIST()
    
    
    #Checking constraints      
    def checkAngleValues(self):
        if(abs(vectorspace().theta_angles[0]) > np.deg2rad(185)):
            return False
        if(vectorspace().theta_angles[1] < np.deg2rad(-135)):
            return False
        if(vectorspace().theta_angles[1] > np.deg2rad(35)):
            return False
        if(vectorspace().theta_angles[2] < np.deg2rad(-120)):
            return False
        if(vectorspace().theta_angles[2] > np.deg2rad(158)):
            return False
        if(abs(self.theta_angles[3]) > np.deg2rad(350)):
            return False
        if(abs(self.theta_angles[4]) > np.deg2rad(130)):
            return False
        if(abs(self.theta_angles[5]) > np.deg2rad(350)):
            return False
        return True

    def __str__(self):
        return"("+",".join(["%.2f"%np.rad2deg(a) for a in self.theta_angles])+")"
    
    
        
   
class transformation(object):
    #inintialization
    D=0
    d34=0
    gamma=0
    beta=0
    
    DH_d = vectorspace().DH_d
    DH_a = vectorspace().DH_a
    DH_alpha = vectorspace().DH_alpha

    def __init__(self,matrix=np.eye(4,4, 0, dtype=float64),d=0,a=0,alpha=0,theta=0):
        self.matrix=matrix
        self.rotate_z(theta)
        self.translate([0,0,d])
        self.rotate_x(alpha)
        self.translate([a,0,0])
     
    def transform(self,matrix):
        self.matrix=np.dot(matrix,self.matrix)
        return self
 
    def translate(self, vector3):
   
        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][3] = vector3[0]
        transform[1][3] = vector3[1]
        transform[2][3] = vector3[2]
        return self.transform(transform)


    def rotate_x(self, angle):

        transform = np.eye(4, 4,dtype=np.float64)
        transform[1][1] = cos(angle)
        transform[1][2] = -1 * sin(angle)
        transform[2][1] = sin(angle)
        transform[2][2] = cos(angle)
        return self.transform(transform)


    def rotate_y(self, angle):
    
        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][0] = cos(angle)
        transform[0][2] = -1 * sin(angle)
        transform[2][0] = sin(angle)
        transform[2][2] = cos(angle)
        return self.transform(transform)


    def rotate_z(self, angle):

        transform = np.eye(4, 4,dtype=np.float64)
        transform[0][0] = cos(angle)
        transform[0][1] = -1 * sin(angle)
        transform[1][0] = sin(angle)
        transform[1][1] = cos(angle)
        return self.transform(transform)
    
    #to maintain the inverse also homogeneous matrix
    #Creating problem when getting IKsolutions as error was scalar terms cannot be given
    def inv(self):
        inverse = np.linalg.inv(self.matrix)
        return transformation(matrix=inverse)

 
    def __mul__(self, other):
        return transformation(matrix=np.dot(self.matrix, other.matrix))
    
    def __str__(self):
        return str(self.matrix)
    #DH_alpha = vectorspace().DH_alpha
    #function return the A0_TCP matrix
    #A0_6 is having forming roatation and postion matrix            
    #def A0TCP(self,X,g forming roatation and postion matrix
    def Postion(self):
        return (self.matrix[0][3],self.matrix[1][3],self.matrix[2][3])
    def Orientation(self):
        return (self.matrix[0][2],self.matrix[1][2],self.matrix[2][2])
          
    def WCP(self):
         global Awcp
         vectorspaced=vectorspace()
         Awcp=self*vectorspaced.base26()
         global wcp1
         poswcp=Awcp.Postion()
         #print "wcp",wcp
         Oriwcp=Awcp.Orientation()
         #print "Oriwcp",Oriwcp
         for i in range(3):
             wcp1[i]=poswcp[i]-(DH_d[6]*Oriwcp[i])
         if(wcp1[1]==0 and wcp1[2]==0):  #shoulder singularity
             wcp[1]=0.000001
         return wcp1
#         global Awcp
#         vectorspaced=vectorspace()
#         Awcp=self*vectorspaced.base26() 
#         global wcp
#         wcp=Awcp.Postion()[0:3]
#         if(wcp[1]==0 and wcp[2]==0):  #shoulder singularity
#            wcp[1]=0.000001
#         return wcp
         
         #if(wcp[1]==0 and wcp[2]==0):  #shoulder singularity
        #   wcp[1]=0.000001
     
    
    
    #Euler angle calculation
    def EulerAngle(self):
        phi=0
        psi=0
        theta=0
        if(self.matrix[2][0]!=1 or self.matrix[2][0]!=-1):
            theta = asin(self.matrix[2][0])
            costheta = math.cos(theta)
            psi = math.atan2(self.matrix[1][0] /costheta , self.matrix[0][0] /costheta )
            phi = math.atan2(self.matrix[2][1] /costheta , self.matrix[2][2] /costheta )
        else:
            phi=0   
            if(self.matrix[2][0]!=-1):
                theta=pi/2
                psi=phi+atan2(self[0][1],self[0][2])
            else:
                theta=-1*pi/2
                psi=(-1*phi)+atan2(-1*A0_6[0][1],-1*A0_6[0][2])
            return(phi,theta,phi)
        return(phi,theta,phi)
    def cart(self):
        cv = "%.2f;" % self.matrix[0][3]
        cv = cv + "%.2f;" % self.matrix[1][3]
        cv = cv + "%.2f;" % self.matrix[2][3]
        cv = cv + "%.2f;" % np.rad2deg(self.EulerAngle()[0])
        cv = cv + "%.2f;" % np.rad2deg(self.EulerAngle()[1])
        cv = cv + "%.2f;" % np.rad2deg(self.EulerAngle()[2])
        print(cv)
        return cv
    
    #function @return Q1_F =theta1 front configuration,Q1_R=theta1 front arm configuration  
    def getQ1(self,xc,yc):
          global Q1_F,Q1_R
          Q1_F = math.atan2(yc,xc)
          Q2_R = math.atan2(yc,xc) + math.pi
          return [Q1_F, Q2_R]
     
   
    #function
    #@param wrist center point values
    #@return Q2_FEUp=theta2 front Elbow Up configuration,Q2_FEDown=theta2 front Elbow Down arm configuration      
    def getQ2Front(self,xc,yc,zc):
            global DH_a,DH_d,Q2_FEUp,Q2_FEDown
            xy=math.sqrt((yc)**2+(xc)**2)
            d34 =math.sqrt((DH_a[2])**2+(DH_d[3])**2)
            gamma=math.atan((abs(zc)-abs(DH_d[0]))/(xy - DH_a[0]))
            dFront=math.sqrt((xy - DH_a[0])**2 + (abs(zc) - abs(DH_d[0]))**2)
            beta_F=math.acos(((-1*d34*d34)+(DH_a[1]**2)+(dFront**2))/(2.0*DH_a[1]*dFront))
            Q2_FEUp=(beta_F + gamma) - (math.pi/2.0)       #front elbow up
            Q2_FEDown = (gamma - beta_F) - (math.pi/2.0)   #front elbow down   
            return [Q2_FEUp,Q2_FEDown]
   
     #function @return Q2_REUp=theta2 Rear Elbow Up configuration,Q2_REDown=theta2 Rear Elbow Down arm configuration
    def getQ2Rear(self,xc,yc,zc):
            global DH_a,DH_d,Q2_REUp,Q2_REDown
            xy=math.sqrt((yc)**2+(xc)**2)
            d34=math.sqrt((DH_a[2])**2+(DH_d[3])**2)
            gamma=math.atan((zc-abs(DH_d[0]))/(xy - DH_a[0]))
            dRear=math.sqrt((xy + DH_a[0])**2 + (abs(zc) - abs(DH_d[0]))**2)
            beta_R=math.acos(((-1*d34*d34)+(DH_a[1]**2)+(dRear**2))/(2.0*DH_a[1]*dRear))
            Q2_REUp=(math.pi/2.0)-(beta_R+gamma)
            Q2_REDown=(math.pi/2.0)-(gamma-beta_R)
            return [Q2_REUp,Q2_REDown]
       
    #function @return Q3_FEUp=theta3 front Elbow Up configuration,Q3_FEDown=theta2 front Elbow Down arm configuration         
    def getQ3Front(self,xc,yc,zc):
            global DH_a,DH_d,Q3_FEUP,Q3_FEDown
            xy=math.sqrt((yc)**2+(xc)**2)
            d34=math.sqrt((DH_a[2])**2+(DH_d[3])**2)
            gamma=math.atan((zc-abs(DH_d[0]))/(xy - DH_a[0]))
            dFront=math.sqrt((xy - DH_a[0])**2 + (abs(zc) - abs(DH_d[0]))**2)
            delta = math.atan(abs(DH_d[3]) / DH_a[2])
            eta_F=acos(((-1*dFront**2)+(d34**2)+(1200**2))/(2*d34*1200))
            Q3_FEUP=eta_F+delta+math.pi
            Q3_FEDown=math.pi-(eta_F-delta)
            return [Q3_FEUp,Q3_FEDown]
   
     #function @return Q3_REUp=theta3 front Elbow Up configuration,Q3_REDown=theta2 front Elbow Down arm configuration         
    def getQ3Rear(self,xc,yc,zc):
            global DH_a,DH_d,Q3_REUp,Q3_REDown
            xy=math.sqrt((yc)**2+(xc)**2)
            d34=math.sqrt((DH_a[2])**2+(DH_d[3])**2)
            gamma=math.atan((zc-abs(DH_d[0]))/(xy - DH_a[0]))
            dRear=math.sqrt((xy + DH_a[0])**2 + (abs(xc) - abs(DH_d[0]))**2)
            delta = math.atan(abs(DH_d[3]) / DH_a[2])
            eta_R=acos(((-1*dRear**2)+(d34**2)+(1200**2))/(2*d34*1200))
            Q3_REUp=(math.pi/2-(eta_R-delta))+(math.pi/2)
            Q3_REDown=(math.pi/2)-((1.5*math.pi)-delta-eta_R)
            return [Q3_REUp,Q3_REDown]
   
     #function Calculate angles 4 possibilities  
    
   
    
    def CalculateAngles(self,wcp):
            IKPosResults = []
            q1 = self.getQ1(wcp[0],wcp[1])   
            q21 = self.getQ2Front(wcp[0],wcp[1],wcp[2])
            q31 = self.getQ3Front(wcp[0],wcp[1],wcp[2])
            arr1 = (q1[0], q21[0], q31[0])
            arr2 = (q1[1], q21[1], q31[1])
            q22 = self.getQ2Rear(wcp[0],wcp[1],wcp[2])
            q32 = self.getQ3Rear(wcp[0],wcp[1],wcp[2])
            arr3 = (q1[0], q22[0], q32[0])
            arr4 = (q1[1], q22[1], q32[1])
            IKPosResults.append(arr1)
            IKPosResults.append(arr2)
            IKPosResults.append(arr3)
            IKPosResults.append(arr4)
            return IKPosResults
       #orientation of wrist theta4,theta5,theta6 having 2 possibilities
    def ik_orientation(self):
        sintheta=np.sqrt(1-((self.matrix[2][2]**2)))
        theta5=atan2((sintheta),self.matrix[2][2])
        if(theta5==0 and self.matrix[0][2]==0):
            theta4 = np.arctan2(self.matrix[1][2], self.matrix[0][2]+0.0001)
        else:
            theta4 = np.arctan2(self.matrix[1][2], self.matrix[0][2])
        theta6 = np.arctan2(self.matrix[2][1], -1 * self.matrix[2][0])
        
        solution1 = (theta4, theta5, theta6)
        solution2 = (theta4+math.pi, -1 * theta5, theta6 + math.pi)#reverse solution
        return [solution1, solution2]
    
    #Function IKSolutions
    def IKSolution(self):
         global wcp1
         wcp=self.WCP()
#          global Awcp
#          vectorspaced=vectorspace()
#          Awcp=self*vectorspaced.base26()
#          global wcp
#          poswcp=Awcp.Postion()
#          #print "wcp",wcp
#          Oriwcp=Awcp.Orientation()
#          #print "Oriwcp",Oriwcp
#          for i in range(3):
#              wcp[i]=poswcp[i]-(DH_d[6]*Oriwcp[i])
         solutions=self.CalculateAngles(wcp)
         IKSol=[]
         for i in solutions:
            vs = vectorspace(theta_angles=i+(0,0,0))
            Awrist = vs.baseToWrist().inv()* self * vs.A6tcp().inv()
           
            wristJoints = Awrist.ik_orientation()        
            IKSol =IKSol+ [vectorspace(theta_angles=i+wristJoints[0])]   # thet1-theta6 possible solution
            IKSol = IKSol + [vectorspace(theta_angles=i+wristJoints[1])]
            for thetas in IKSol:
                if(thetas.checkAngleValues()):   #thata1-theta6 possible solution
                    return IKSol

        
if __name__ == '__main__':
    vector=vectorspace()
    #basetotool=vector.A6tcp().matrix
    #print basetotool
    tranform=transformation()
    ik=tranform.IKSolution()
    print ik
    #rotate_x=rotatex.rotate_x(4).matrix
    #print rota
    #inverse=InverseKinematics([1,1,1],[0,0,0])


