#!/usr/bin/env python3
############################################################
## BRUFACE: Brussels Faculty of Engineering 
############################################################
## Master Thesis " Autonmous Grabbing with RGB-D"  
## July 2022 Manar Mahmalji      
## Version (1)     
############################################################
## This code serves for implementing an intermittent Kalman Filter 
# that sends a corrected pose to the robot if it receives data from 
# the camera; otherwise, it sends predicted pose based on the 5 DOF
# suspended block model 

import rospy
from std_msgs.msg  import Float64MultiArray
import time 
import numpy as np
import math as ma 


# Global variables 
first_time= True 
no_interrupt= False 
temp = False 
corrected= False
cor_flag= False
first_data_camera= False
first_data_camera2= False

T_mat= Float64MultiArray() # instantiate an object
current_pose= 0 

# Linearized model
##################CHANGE_IF_NEEDED##################
A = np.loadtxt('/home/utente/Desktop/Ad_2.txt')# 10x10
##################CHANGE_IF_NEEDED##################
C= np.zeros((5,10)) # 10x10 
C[0,0] = 1
C[1,1] = 1
C[2,2] = 1
C[3,3] = 1
C[4,4] = 1





# KF paramters 
##################CHANGE_IF_NEEDED##################
R= np.eye(12)*10# noise 12x12
#R[1,1] = 100
#R= np.eye(5)


Q= np.eye(10)*0.001 # disturance  10x10


B= np.ones((10,1))

##################CHANGE_IF_NEEDED##################
K= np.zeros((10,12)) # gain 10x10  
#K= np.zeros((10,5)) # gain 10x10  
error= np.zeros((12,1)) # 10x1 
#error= np.zeros(5) # 10x1 

x_next= np.zeros((10,1)) # 10x1
x_cam= np.zeros((12,1)) # 10x1  
p_next= np.zeros((10,10)) # 10x10

x_curr= np.zeros((10,1)) # initial condition 
x_curr[0] = 0*3.14/180
p_curr= np.eye(10)*10 # we don't trust our initial condition





# Go from rotation matrix R and translation vector t to T matrix
def matrix_from_rt(R,t):
    M = np.eye(4)
    M[0:3, 0:3] = R
    M[0:3, 3] = t.squeeze()
    return (M)

# Rotation matrix around z 
def Rotz(gamma):
    R= np.array([[ma.cos(gamma), -ma.sin(gamma), 0],
                [ma.sin(gamma),  ma.cos(gamma), 0],
                [0, 0, 1]])  
    return R


def rt_from_matrix(M):
    r= M[0:3, 0:3] 
    t=  M[0:3, 3]
    return (r, t)

def dir_kin(q,l,b):
    q1 = q[0][0]
    q2 = q[1][0]
    q3 = q[2][0]
    q4 = q[3][0]
    q5 = q[4][0]


    #  T matrix from the fixed frame to the spherical joint
    T_02 = np.array([[ ma.cos(q2)*ma.sin(q1), -ma.sin(q1)*ma.sin(q2),  ma.cos(q1), -l*ma.cos(q2)*ma.sin(q1)],
            [-ma.sin(q2),         -ma.cos(q2),        0,          l*ma.sin(q2)],
            [ma.cos(q1)*ma.cos(q2), -ma.cos(q1)*ma.sin(q2), -ma.sin(q1), -l*ma.cos(q1)*ma.cos(q2)],
            [0,                0,        0,                  1]])
    
    T_25 = np.array([[ - ma.sin(q3)*ma.sin(q5) - ma.cos(q3)*ma.cos(q5)*ma.sin(q4), ma.cos(q3)*ma.sin(q4)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3), -ma.cos(q3)*ma.cos(q4),-(b*10*ma.cos(q3)*ma.cos(q4))/10],
            [ ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4), ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5), -ma.cos(q4)*ma.sin(q3), -(b*10*ma.cos(q4)*ma.sin(q3))/10],
            [ ma.cos(q4)*ma.cos(q5), -ma.cos(q4)*ma.sin(q5), -ma.sin(q4),-(b*10*ma.sin(q4))/10],
            [0, 0,  0,  1]])


    T= np.dot(T_02, T_25)

    return T

def dir_kin_vec(q,l,b):
    q1 = q[0][0]
    q2 = q[1][0]
    q3 = q[2][0]
    q4 = q[3][0]
    q5 = q[4][0]

    x_h = np.zeros((12,1))

    #  T matrix from the fixed frame to the spherical joint
    T_02 = np.array([[ ma.cos(q2)*ma.sin(q1), -ma.sin(q1)*ma.sin(q2),  ma.cos(q1), -l*ma.cos(q2)*ma.sin(q1)],
            [-ma.sin(q2),         -ma.cos(q2),        0,          l*ma.sin(q2)],
            [ma.cos(q1)*ma.cos(q2), -ma.cos(q1)*ma.sin(q2), -ma.sin(q1), -l*ma.cos(q1)*ma.cos(q2)],
            [0,                0,        0,                  1]])
    
    T_25 = np.array([[ - ma.sin(q3)*ma.sin(q5) - ma.cos(q3)*ma.cos(q5)*ma.sin(q4), ma.cos(q3)*ma.sin(q4)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3), -ma.cos(q3)*ma.cos(q4),-(b*10*ma.cos(q3)*ma.cos(q4))/10],
            [ ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4), ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5), -ma.cos(q4)*ma.sin(q3), -(b*10*ma.cos(q4)*ma.sin(q3))/10],
            [ ma.cos(q4)*ma.cos(q5), -ma.cos(q4)*ma.sin(q5), -ma.sin(q4),-(b*10*ma.sin(q4))/10],
            [0, 0,  0,  1]])


    T= np.dot(T_02, T_25)

    [R_temp, t_temp]= rt_from_matrix(T)
    t1= t_temp[0]
    t2= t_temp[1]
    t3= t_temp[2]
    r11= R_temp[0][0]
    r12= R_temp[0][1]
    r13= R_temp[0][2]
    r21= R_temp[1][0]
    r22= R_temp[1][1]
    r23= R_temp[1][2]
    r31= R_temp[2][0]
    r32= R_temp[2][1]
    r33= R_temp[2][2]
    
    x_h = np.array([t1,t2,t3,r11,r12,r13,r21,r22,r23,r31,r32,r33])

    return x_h

def jacobian_out(q,l,b):

    q1 = q[0][0]
    q2 = q[1][0]
    q3 = q[2][0]
    q4 = q[3][0]
    q5 = q[4][0]

    H = np.zeros((12,10))
    H[0,0] = b*ma.sin(q1)*ma.sin(q4) - l*ma.cos(q1)*ma.cos(q2) - b*ma.cos(q1)*ma.cos(q2)*ma.cos(q3)*ma.cos(q4) + b*ma.cos(q1)*ma.cos(q4)*ma.sin(q2)*ma.sin(q3)
    H[0,1] = l*ma.sin(q1)*ma.sin(q2) + b*ma.cos(q2)*ma.cos(q4)*ma.sin(q1)*ma.sin(q3) + b*ma.cos(q3)*ma.cos(q4)*ma.sin(q1)*ma.sin(q2)
    H[0,2] = b*ma.cos(q2)*ma.cos(q4)*ma.sin(q1)*ma.sin(q3) + b*ma.cos(q3)*ma.cos(q4)*ma.sin(q1)*ma.sin(q2)
    H[0,3] = b*ma.cos(q2)*ma.cos(q3)*ma.sin(q1)*ma.sin(q4) - b*ma.cos(q1)*ma.cos(q4) - b*ma.sin(q1)*ma.sin(q2)*ma.sin(q3)*ma.sin(q4)
    H[0,4] = 0
    H[0,5] = 0
    H[0,6] = 0
    H[0,7] = 0
    H[0,8] = 0
    H[0,9] = 0
    H[1,0] = 0
    H[1,1] = l*ma.cos(q2) + b*ma.cos(q2)*ma.cos(q3)*ma.cos(q4) - b*ma.cos(q4)*ma.sin(q2)*ma.sin(q3)
    H[1,2] = b*ma.cos(q2)*ma.cos(q3)*ma.cos(q4) - b*ma.cos(q4)*ma.sin(q2)*ma.sin(q3)
    H[1,3] = - b*ma.cos(q2)*ma.sin(q3)*ma.sin(q4) - b*ma.cos(q3)*ma.sin(q2)*ma.sin(q4)
    H[1,4] = 0
    H[1,5] = 0
    H[1,6] = 0
    H[1,7] = 0
    H[1,8] = 0
    H[1,9] = 0
    H[2,0] = b*ma.cos(q1)*ma.sin(q4) + l*ma.cos(q2)*ma.sin(q1) + b*ma.cos(q2)*ma.cos(q3)*ma.cos(q4)*ma.sin(q1) - b*ma.cos(q4)*ma.sin(q1)*ma.sin(q2)*ma.sin(q3)
    H[2,1] = l*ma.cos(q1)*ma.sin(q2) + b*ma.cos(q1)*ma.cos(q2)*ma.cos(q4)*ma.sin(q3) + b*ma.cos(q1)*ma.cos(q3)*ma.cos(q4)*ma.sin(q2)
    H[2,2] = b*ma.cos(q1)*ma.cos(q2)*ma.cos(q4)*ma.sin(q3) + b*ma.cos(q1)*ma.cos(q3)*ma.cos(q4)*ma.sin(q2)
    H[2,3] = b*ma.cos(q4)*ma.sin(q1) + b*ma.cos(q1)*ma.cos(q2)*ma.cos(q3)*ma.sin(q4) - b*ma.cos(q1)*ma.sin(q2)*ma.sin(q3)*ma.sin(q4)
    H[2,4] = 0
    H[2,5] = 0
    H[2,6] = 0
    H[2,7] = 0
    H[2,8] = 0
    H[2,9] = 0
    H[3,0] = - ma.cos(q1)*ma.cos(q2)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) - ma.cos(q1)*ma.sin(q2)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4)) - ma.cos(q4)*ma.cos(q5)*ma.sin(q1)
    H[3,1] = ma.sin(q1)*ma.sin(q2)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) - ma.cos(q2)*ma.sin(q1)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4))
    H[3,2] = ma.sin(q1)*ma.sin(q2)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) - ma.cos(q2)*ma.sin(q1)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4))
    H[3,3] = ma.cos(q4)*ma.cos(q5)*ma.sin(q1)*ma.sin(q2)*ma.sin(q3) - ma.cos(q2)*ma.cos(q3)*ma.cos(q4)*ma.cos(q5)*ma.sin(q1) - ma.cos(q1)*ma.cos(q5)*ma.sin(q4)
    H[3,4] = - ma.cos(q2)*ma.sin(q1)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5)) - ma.sin(q1)*ma.sin(q2)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5)) - ma.cos(q1)*ma.cos(q4)*ma.sin(q5)
    H[3,5] = 0
    H[3,6] = 0
    H[3,7] = 0
    H[3,8] = 0
    H[3,9] = 0
    H[4,0] = ma.cos(q4)*ma.sin(q1)*ma.sin(q5) - ma.cos(q1)*ma.sin(q2)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5)) - ma.cos(q1)*ma.cos(q2)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5))
    H[4,1] = ma.sin(q1)*ma.sin(q2)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5)) - ma.cos(q2)*ma.sin(q1)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5))
    H[4,2] = ma.sin(q1)*ma.sin(q2)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5)) - ma.cos(q2)*ma.sin(q1)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5))
    H[4,3] = ma.cos(q1)*ma.sin(q4)*ma.sin(q5) + ma.cos(q2)*ma.cos(q3)*ma.cos(q4)*ma.sin(q1)*ma.sin(q5) - ma.cos(q4)*ma.sin(q1)*ma.sin(q2)*ma.sin(q3)*ma.sin(q5)
    H[4,4] = ma.cos(q2)*ma.sin(q1)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) + ma.sin(q1)*ma.sin(q2)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4)) - ma.cos(q1)*ma.cos(q4)*ma.cos(q5)
    H[4,5] = 0
    H[4,6] = 0
    H[4,7] = 0
    H[4,8] = 0
    H[4,9] = 0
    H[5,0] = ma.sin(q1)*ma.sin(q4) - ma.cos(q1)*ma.cos(q2)*ma.cos(q3)*ma.cos(q4) + ma.cos(q1)*ma.cos(q4)*ma.sin(q2)*ma.sin(q3)
    H[5,1] = ma.cos(q2)*ma.cos(q4)*ma.sin(q1)*ma.sin(q3) + ma.cos(q3)*ma.cos(q4)*ma.sin(q1)*ma.sin(q2)
    H[5,2] = ma.cos(q2)*ma.cos(q4)*ma.sin(q1)*ma.sin(q3) + ma.cos(q3)*ma.cos(q4)*ma.sin(q1)*ma.sin(q2)
    H[5,3] = ma.cos(q2)*ma.cos(q3)*ma.sin(q1)*ma.sin(q4) - ma.cos(q1)*ma.cos(q4) - ma.sin(q1)*ma.sin(q2)*ma.sin(q3)*ma.sin(q4)
    H[5,4] = 0
    H[5,5] = 0
    H[5,6] = 0
    H[5,7] = 0
    H[5,8] = 0
    H[5,9] = 0
    H[6,0] = 0
    H[6,1] = ma.cos(q2)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) + ma.sin(q2)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4))
    H[6,2] = ma.cos(q2)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) + ma.sin(q2)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4))
    H[6,3] = ma.cos(q2)*ma.cos(q4)*ma.cos(q5)*ma.sin(q3) + ma.cos(q3)*ma.cos(q4)*ma.cos(q5)*ma.sin(q2)
    H[6,4] = ma.sin(q2)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5)) - ma.cos(q2)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5))
    H[6,5] = 0
    H[6,6] = 0
    H[6,7] = 0
    H[6,8] = 0
    H[6,9] = 0
    H[7,0] = 0
    H[7,1] = ma.cos(q2)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5)) + ma.sin(q2)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5))
    H[7,2] = ma.cos(q2)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5)) + ma.sin(q2)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5))
    H[7,3] = - ma.cos(q2)*ma.cos(q4)*ma.sin(q3)*ma.sin(q5) - ma.cos(q3)*ma.cos(q4)*ma.sin(q2)*ma.sin(q5)
    H[7,4] = ma.cos(q2)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4)) - ma.sin(q2)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4))
    H[7,5] = 0
    H[7,6] = 0
    H[7,7] = 0
    H[7,8] = 0
    H[7,9] = 0
    H[8,0] = 0
    H[8,1] = ma.cos(q2 + q3)*ma.cos(q4)
    H[8,2] = ma.cos(q2 + q3)*ma.cos(q4)
    H[8,3] = -ma.sin(q2 + q3)*ma.sin(q4)
    H[8,4] = 0
    H[8,5] = 0
    H[8,6] = 0
    H[8,7] = 0
    H[8,8] = 0
    H[8,9] = 0
    H[9,0] = ma.cos(q2)*ma.sin(q1)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) + ma.sin(q1)*ma.sin(q2)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4)) - ma.cos(q1)*ma.cos(q4)*ma.cos(q5)
    H[9,1] = ma.cos(q1)*ma.sin(q2)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) - ma.cos(q1)*ma.cos(q2)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4))
    H[9,2] = ma.cos(q1)*ma.sin(q2)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) - ma.cos(q1)*ma.cos(q2)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4))
    H[9,3] = ma.cos(q5)*ma.sin(q1)*ma.sin(q4) - ma.cos(q1)*ma.cos(q2)*ma.cos(q3)*ma.cos(q4)*ma.cos(q5) + ma.cos(q1)*ma.cos(q4)*ma.cos(q5)*ma.sin(q2)*ma.sin(q3)
    H[9,4] = ma.cos(q4)*ma.sin(q1)*ma.sin(q5) - ma.cos(q1)*ma.sin(q2)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5)) - ma.cos(q1)*ma.cos(q2)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5))
    H[9,5] = 0
    H[9,6] = 0
    H[9,7] = 0
    H[9,8] = 0
    H[9,9] = 0
    H[10,0] = ma.cos(q2)*ma.sin(q1)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5)) + ma.sin(q1)*ma.sin(q2)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5)) + ma.cos(q1)*ma.cos(q4)*ma.sin(q5)
    H[10,1] = ma.cos(q1)*ma.sin(q2)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5)) - ma.cos(q1)*ma.cos(q2)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5))
    H[10,2] = ma.cos(q1)*ma.sin(q2)*(ma.cos(q5)*ma.sin(q3) - ma.cos(q3)*ma.sin(q4)*ma.sin(q5)) - ma.cos(q1)*ma.cos(q2)*(ma.cos(q3)*ma.cos(q5) + ma.sin(q3)*ma.sin(q4)*ma.sin(q5))
    H[10,3] = ma.cos(q1)*ma.cos(q2)*ma.cos(q3)*ma.cos(q4)*ma.sin(q5) - ma.sin(q1)*ma.sin(q4)*ma.sin(q5) - ma.cos(q1)*ma.cos(q4)*ma.sin(q2)*ma.sin(q3)*ma.sin(q5)
    H[10,4] = ma.cos(q1)*ma.cos(q2)*(ma.sin(q3)*ma.sin(q5) + ma.cos(q3)*ma.cos(q5)*ma.sin(q4)) + ma.cos(q1)*ma.sin(q2)*(ma.cos(q3)*ma.sin(q5) - ma.cos(q5)*ma.sin(q3)*ma.sin(q4)) + ma.cos(q4)*ma.cos(q5)*ma.sin(q1)
    H[10,5] = 0
    H[10,6] = 0
    H[10,7] = 0
    H[10,8] = 0
    H[10,9] = 0
    H[11,0] = ma.cos(q1)*ma.sin(q4) + ma.cos(q2)*ma.cos(q3)*ma.cos(q4)*ma.sin(q1) - ma.cos(q4)*ma.sin(q1)*ma.sin(q2)*ma.sin(q3)
    H[11,1] = ma.cos(q1)*ma.cos(q2)*ma.cos(q4)*ma.sin(q3) + ma.cos(q1)*ma.cos(q3)*ma.cos(q4)*ma.sin(q2)
    H[11,2] = ma.cos(q1)*ma.cos(q2)*ma.cos(q4)*ma.sin(q3) + ma.cos(q1)*ma.cos(q3)*ma.cos(q4)*ma.sin(q2)
    H[11,3] = ma.cos(q4)*ma.sin(q1) + ma.cos(q1)*ma.cos(q2)*ma.cos(q3)*ma.sin(q4) - ma.cos(q1)*ma.sin(q2)*ma.sin(q3)*ma.sin(q4)
    H[11,4] = 0
    H[11,5] = 0
    H[11,6] = 0
    H[11,7] = 0
    H[11,8] = 0
    H[11,9] = 0
    
    
    
    return H


def callback(data):
    global cor_flag
    global x_cam
    global first_data_camera
    
    cor_flag= True
    first_data_camera= True
    x_cam= np.asarray(data.data) # 1x5
    #print('correction',x_cam)

            



def listener():
    ##################CHANGE_IF_NEEDED##################
    rospy.init_node('Interm_KF', anonymous=False)
    pub = rospy.Publisher('ArUcoPose', Float64MultiArray, queue_size=1)
    #pub1 = rospy.Publisher('ArUcoPose', Float64MultiArray, queue_size=1)
    rospy.Subscriber("ArUcoPose_D455", Float64MultiArray, callback)
    ##################CHANGE_IF_NEEDED##################
    pose= Float64MultiArray()

     
    ##################CHANGE_IF_NEEDED##################
    identifier = 1 # to know when correction took place

    ##################CHANGE_IF_NEEDED##################
    l = 1.215;  #length cable(m)
    b = 0.3;  #height_block/2 (m)
    ##################CHANGE_IF_NEEDED##################
    
    # Get rotation matrix of block frame wrt crane frame
    #  
    # First we define the crane frame 
    ##################CHANGE_IF_NEEDED#################
    gamma= 43*( np.pi/180) 
    # transaltion (in mm)

    ##################CHANGE_IF_NEEDED##################
    x_sensor = 410 # this is what x_sensor reads 
    ##################CHANGE_IF_NEEDED##################
    x= 2450-830-340-280-x_sensor + 60 #2450(the distance structure,fix), 830
    y= 410 -50 # last term is tuning 
    z= -2255 #fix from the top to the wood plate (-2235)
    ##################CHANGE_IF_NEEDED##################
    R_base2crane= Rotz(gamma)
    t_base2crane= np.array([x*0.001,y*0.001,z*0.001])

    T_crane2base= matrix_from_rt(R_base2crane.T,-R_base2crane.T @ t_base2crane) 
    #print("pos")
    #print(R_base2crane)

    R_gr2block= Rotz(0)
    t_gr2block= np.array([450*0.001,y*0,z*0])

    T_gr2block= matrix_from_rt(R_gr2block,t_gr2block)


    R_rel= np.zeros((3,3)) #Roty(pi/2)*Rotx(-pi)
    R_rel[2,0] = -1
    R_rel[1,1] = -1
    R_rel[0,2] = -1

    t_rel= np.array([0.1520,-0.0174,0.0173])

    T_rel= matrix_from_rt(R_rel,t_rel)

    #print("T_Rel")
    #print(T_rel)
      



    ######################### start code KF #################
    global A
    global Q
    global B
    global K
    global p
    global x_curr
    global p_curr
    global x_next
    global p_next
    global error 
    global first_time 
    global no_interrupt
    global temp 
    global K_temp 
    global error_temp 
    global corrected
    global first_data_camera 
    global cor_flag
    global x_cam
    
    q = np.zeros((5,1))
    out_sys = np.zeros((12,1))
    pose_read = np.zeros((50000,12))
    T_fin = np.zeros((4,4))
    
    k= 0
    #H = jacobian_out(q,l,b)
     
    #print("H")
    #print(H)

    mex = 2
    n_msg = 0

    noise_x= np.random.normal(0,0.005,10)
    rate= rospy.Rate(200) # Hz
    iter = 0

    while not rospy.is_shutdown():
        start= time.time()
        #prediction
        print("prediction")
        x_curr= A @ x_curr #+ noise_x
        p_curr= A @ p_curr @ (A.T) + Q

        
        if cor_flag:
            print("correction")
            identifier= 2
            q = x_curr[0:5]
            out_sys = dir_kin_vec(q,l,b)
            H = jacobian_out(q,l,b)
            K= p_curr @ (H.T) @ np.linalg.inv( H @ p_curr @ H.T + R) #10x5
            error[0] = x_cam[0] - out_sys[0]
            error[1] = x_cam[1] - out_sys[1]
            error[2] = x_cam[2] - out_sys[2]
            error[3] = x_cam[3] - out_sys[3]
            error[4] = x_cam[4] - out_sys[4]
            error[5] = x_cam[5] - out_sys[5]
            error[6] = x_cam[6] - out_sys[6]
            error[7] = x_cam[7] - out_sys[7]
            error[8] = x_cam[8] - out_sys[8]
            error[9] = x_cam[9] - out_sys[9]
            error[10] = x_cam[10] - out_sys[10]
            error[11] = x_cam[11] - out_sys[11]

            x_curr= x_curr + K @ error
            p_curr= p_curr - K @ H @ p_curr

        else:
            #print("no correction")
            identifier= 1
            x_curr= x_curr
            p_curr= p_curr
        cor_flag= False

        q = x_curr[0:5]
        out_sys = dir_kin_vec(q,l,b)

        #print("out",out_sys)
        
        t1 = out_sys[0]
        t2 = out_sys[1]
        t3 = out_sys[2]
        #pose.data= np.array([identifier,t1,t2,t3,0,0,0,0,0,0,0,0,0,0])
        #pub.publish(pose)

        T_fin= dir_kin(q,l,b)
        T_2fin= T_crane2base @ T_fin @ T_gr2block @ T_rel
        if first_data_camera:
            current_pose = np.array([mex,T_2fin[0,3],T_2fin[1,3],T_2fin[2,3],T_2fin[0,0],T_2fin[0,1],T_2fin[0,2],T_2fin[1,0],T_2fin[1,1],T_2fin[1,2],T_2fin[2,0],T_2fin[2,1],T_2fin[2,2]])
            T_mat.data=current_pose
            #print("pose:",current_pose)
            pub.publish(T_mat)
            if n_msg > 100:
                mex = 1
            n_msg = n_msg + 1
            #print("n_msg",n_msg)

        #try to implement a MA

        if first_data_camera2:
            pose_read[k][0]= out_sys[0]
            pose_read[k][1]= out_sys[1]
            pose_read[k][2]= out_sys[2]
            pose_read[k][3]= out_sys[3]
            pose_read[k][4]= out_sys[4]
            pose_read[k][5]= out_sys[5]
            pose_read[k][6]= out_sys[6]
            pose_read[k][7]= out_sys[7]
            pose_read[k][8]= out_sys[8]
            pose_read[k][9]= out_sys[9]
            pose_read[k][10]= out_sys[10]
            pose_read[k][11]= out_sys[11]

            print("I'm collecting")
            mov_avg_size= 10

            if k>(mov_avg_size+1):
                _t1=  np.mean(pose_read[k-mov_avg_size:k+1,0])
                _t2=  np.mean(pose_read[k-mov_avg_size:k+1,1])
                _t3=  np.mean(pose_read[k-mov_avg_size:k+1,2])
                _r11=  np.mean(pose_read[k-mov_avg_size:k+1,3])
                _r12=  np.mean(pose_read[k-mov_avg_size:k+1,4])
                _r13=  np.mean(pose_read[k-mov_avg_size:k+1,5])
                _r21=  np.mean(pose_read[k-mov_avg_size:k+1,6])
                _r22=  np.mean(pose_read[k-mov_avg_size:k+1,7])
                _r23=  np.mean(pose_read[k-mov_avg_size:k+1,8])
                _r31=  np.mean(pose_read[k-mov_avg_size:k+1,9])
                _r32=  np.mean(pose_read[k-mov_avg_size:k+1,10])
                _r33=  np.mean(pose_read[k-mov_avg_size:k+1,11])
                #print("I'm publishing",_t1,_t2,_t3)
                #T_fin= dir_kin(q,l,b)
                T_fin[0,0]= _r11
                T_fin[0,1]= _r12
                T_fin[0,2]= _r13
                T_fin[0,3]= _t1
                T_fin[1,0]= _r21
                T_fin[1,1]= _r22
                T_fin[1,2]= _r23
                T_fin[1,3]= _t2
                T_fin[2,0]= _r31
                T_fin[2,1]= _r32
                T_fin[2,2]= _r33
                T_fin[2,3]= _t3
                T_fin[3,3]= 1
                

                #T_2fin= T_crane2base @ T_fin @ T_gr2block @ T_rel
                #current_pose = np.array([3,T_2fin[0,3],T_2fin[1,3],T_2fin[2,3],T_2fin[0,0],T_2fin[0,1],T_2fin[0,2],T_2fin[1,0],T_2fin[1,1],T_2fin[1,2],T_2fin[2,0],T_2fin[2,1],T_2fin[2,2]])
                #T_mat.data=current_pose
                #print("T_2fin",T_2fin)
                #pub.publish(T_mat)

                #pose.data= np.array([identifier,_t1,_t2,_t3,0,0,0,0,0,0,0,0,0,0])
                #pub.publish(pose)
            k = k+1

        
        
        rate.sleep()
        print("Frequency:"+ str((time.time()-start)) + ' s')
        
      

  

if __name__ == '__main__':
    listener()
