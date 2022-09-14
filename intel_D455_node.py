#!/usr/bin/env python3
############################################################
## BRUFACE: Brussels Faculty of Engineering 
############################################################
## Master Thesis " Autonmous Grabbing with RGB-D"  
## July 2022 Manar Mahmalji      
## Version (1)     
############################################################
# This ROS node sends the pose of the block frame centered at 
# its cog to an intermittent Kalman Filter 


import rospy
from std_msgs.msg  import Float64MultiArray
from std_msgs.msg  import Float64
import numpy as np
import pandas as pd   
import pyrealsense2 as rs
import cv2.aruco as aruco
import cv2
import math as ma 
import time 

# Go from T matrix to rotation matrix R and translation vector t 
def rt_from_matrix(M):
    r= M[0:3, 0:3] 
    t=  M[0:3, 3]
    return (r, t)

# Go from rotation matrix R and translation vector t to T matrix
def matrix_from_rt(R,t):
    M = np.eye(4)
    M[0:3, 0:3] = R
    M[0:3, 3] = t.squeeze()
    return (M)

# Go from rvec and tvec to T matrix
def matrix_from_rtvec(rvec, tvec):
    (R, jac) = cv2.Rodrigues(rvec) # ignore the jacobian
    M = np.eye(4)
    M[0:3, 0:3] = R
    M[0:3, 3] = tvec.squeeze() # 1-D vector, row vector, column vector, whatever
    return M

# Check if a matrix is a valid rotation matrix
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    if n>1e-3:
        print("Error! Not a rotation matrix")
    return n<1e-3

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB 
def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    sy = ma.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = ma.atan2(R[2,1] , R[2,2])
        y = ma.atan2(-R[2,0], sy)
        z = ma.atan2(R[1,0], R[0,0])
    else :
        x = ma.atan2(-R[1,2], R[1,1])
        y = ma.atan2(-R[2,0], sy)
        z = 0
    x=ma.degrees(x)
    y=ma.degrees(y)
    z=ma.degrees(z)
    return np.array([z, y, x])

# Rotation matrix around z 
def Rotz(gamma):
    R= np.array([[ma.cos(gamma), -ma.sin(gamma), 0],
                [ma.sin(gamma),  ma.cos(gamma), 0],
                [0, 0, 1]])  
    return R
# Rotation matrix around y
def Roty(betta):
    R= np.array([[ma.cos(betta), 0, ma.sin(betta)],
                [0, 1, 0],
                [-ma.sin(betta), 0, ma.cos(betta)]]) 
    return R
# Rotation matrix around x
def Rotx(alpha):
    R= np.array([[1, 0, 0],
                [0, ma.cos(alpha), -ma.sin(alpha)],
                [0, ma.sin(alpha), ma.cos(alpha)]]) 
    return R

def inv_kin(p_e,r_e,l,b):
    # Rot Matfrix (r_e) and the position (p_e) 
    #print("pos")
    #print(p_e)
    pw = p_e - b*r_e[0:3,2]

    q_2 = ma.atan2(pw[1]/l,ma.sqrt(1-(pw[1]/l)**2))
    q_1 = ma.atan2(pw[0]/(-l*ma.cos(q_2)),ma.sqrt(1-(pw[0]/(-l*ma.cos(q_2)))**2))

   # T02 matrix evaluated in q_1 q_2 

    T_02_ = np.array([[ma.cos(q_2)*ma.sin(q_1), -ma.sin(q_1)*ma.sin(q_2),  ma.cos(q_1), -l*ma.cos(q_2)*ma.sin(q_1)],
                [-ma.sin(q_2),          -ma.cos(q_2),         0,           l*ma.sin(q_2)],
                [ma.cos(q_1)*ma.cos(q_2), -ma.cos(q_1)*ma.sin(q_2), -ma.sin(q_1), -l*ma.cos(q_1)*ma.cos(q_2)],
                [0,                  0,         0,              1]])


    [R02,pos02] = rt_from_matrix(T_02_)
    R25 = np.dot(R02.T,r_e)


    rr = R25
    q_3 = (ma.atan2(rr[1,2],rr[0,2])+ma.pi)
    #print(q_3)
    q_4 = ma.atan2(-rr[2,2],ma.sqrt(rr[1,2]**2+rr[0,2]**2))
    q_5 = ma.atan2(-rr[2,1],rr[2,0])

    
    q_ = np.array([q_1, q_2, q_3, q_4, q_5])
    
    for i in range(5):
        if abs(q_[i]-2*ma.pi)< 1e-5: 
            q_[i] = 0
    return q_

 

def talker():
    # publisher setup 
    ##################CHANGE_IF_NEEDED##################
    #pub = rospy.Publisher('ArUcoPose_D455', Float64, queue_size=1)
    pub_x = rospy.Publisher('ArUcoPose_D455', Float64MultiArray, queue_size=1)
    pub_res= rospy.Publisher('ArUcoPose_D455_res', Float64MultiArray, queue_size=1)
    rospy.init_node('IntelD455', anonymous=True)
    ##################CHANGE_IF_NEEDED##################

    ## camera setup 

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    # Configure resolution of depth and color sensors
    ##################CHANGE_IF_NEEDED##################
    # Lower FPS, higher resoultion 
    # config.enable_stream(rs.stream.color, 1280 , 720, rs.format.bgr8, 30)
    # config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    # Higher FPS, lower resolution 
    config.enable_stream(rs.stream.color, 848 ,480, rs.format.bgr8, 60)
    config.enable_stream(rs.stream.depth, 848 ,480, rs.format.z16, 60)

    # Higher FPS, lower resolution 
    # config.enable_stream(rs.stream.color, 640 ,360, rs.format.bgr8, 90)
    # config.enable_stream(rs.stream.depth, 640 ,360, rs.format.z16, 90)
    ##################CHANGE_IF_NEEDED##################

    if device_product_line == 'D400':
        print('Hello, I am D455')
        
    # Create an align object
    # rs.align allows us to perform alignment of a stream to other streams
    # In this case, depth stream is aligned to color stream since the depth
    # stream is centerd at the left camera whereas the color stream is 
    # centered at the RGB camera/sensor 
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Start streaming
    profile = pipeline.start(config)

    # Information to be saved 
    ##################CHANGE_IF_NEEDED##################
    M= 5000#number of samples
    ##################CHANGE_IF_NEEDED##################


    ##################CHANGE_IF_NEEDED##################
    N=1000 # velocity filter
    ##################CHANGE_IF_NEEDED##################



    # Reading cam2base pose 
    ##################CHANGE_IF_NEEDED##################
    chosen_method= 2 # Park
    ##################CHANGE_IF_NEEDED##################
    cameraPose_filepath='/home/utente/Desktop/Manar_Final/Eye-to-hand Calibration/Calibration Data/cam2base4.xlsx'
    ##################CHANGE_IF_NEEDED##################
    data = pd.read_excel(cameraPose_filepath, sheet_name='method'+str(chosen_method))
    T= data.head()
    T= T.to_numpy()
    (R_cam2base,t_cam2base)= rt_from_matrix(T)


    # Initializations 
    pose= np.zeros((M,6))
    k=0
    q_curr=0 
    q_prev=0 
    first_time= True 
    q_dot=np.array([0,0,0,0,0]) 
    vec_to_send = np.zeros(2)
    data= Float64MultiArray() # instantiate an object
 
    # Inverse kinematics parameters 
    #################CHANGE_IF_NEEDED###################
    l= 1.215
    height=0.3 # height/2
    ##################CHANGE_IF_NEEDED##################

    #################CHANGE_IF_NEEDED###################
    frq= 40 # Hz
    ##################CHANGE_IF_NEEDED##################
    rate = rospy.Rate(frq) # Hz 
    
    start = 0
    start= time.time()
    index= 0
    #my_file = open("data.txt","w")
    #my_file2 = open("time.txt","w")
    while not rospy.is_shutdown():
        start= time.time()
        #print("START")
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get  frames
        color_frame = frames.get_color_frame()
        tmst = frames.get_timestamp()
        depth_frame = aligned_frames.get_depth_frame()
  

    
        if  not color_frame or not depth_frame:
            continue

        # Convert image to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        

        # Get intrinsics for RGB sensor
        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        # Extract camera matrix and distortion coefficients
        mat = [color_intrinsics.fx, 0,color_intrinsics.ppx,0 ,color_intrinsics.fy , color_intrinsics.ppy, 0, 0, 1]
        mat = np.array(mat)
        camera_matrix= mat.reshape((3, 3))
        dist_coeffs= np.array(color_intrinsics.coeffs)


        ## Detection of ArUco Board

        # Detection Parameters 
        arucoParams = aruco.DetectorParameters_create()

        # Tuned paramters. If commented, the paramters are set to default values
        arucoParams.adaptiveThreshWinSizeMin=5
        arucoParams.adaptiveThreshWinSizeMax=20
        arucoParams.adaptiveThreshWinSizeStep=5
        arucoParams.cornerRefinementMethod= aruco.CORNER_REFINE_SUBPIX
        arucoParams.cornerRefinementWinSize=3
        arucoParams.cornerRefinementMinAccuracy=0.01
        arucoParams.cornerRefinementMaxIterations=50
        
        # Board's Parameters 
        ##################CHANGE_IF_NEEDED##################
        aruco_dict = aruco.Dictionary_get( aruco.DICT_5X5_1000 )
        markersX = 5
        markersY = 3
        markerLength = 99 # in mm 
        markerSeparation = 6.8 # in mm 
        ##################CHANGE_IF_NEEDED##################
        board = aruco.GridBoard_create(markersX, markersY, float(markerLength),
                                                    float(markerSeparation), aruco_dict)

        # Detection: we start by detecting all markers 
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, aruco_dict, parameters=arucoParams)  # First, detect markers
        aruco.refineDetectedMarkers(gray_image, board, corners, ids, rejectedImgPoints)


        if len(corners)>0: # if there is at least one marker detected
            # Estimate board pose 
            markNum, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, rvec=np.float32([0,0,0]), tvec=np.float32([0,0,0]))  
            # tvec(in mm) is the 3D postion of the board frame wrt camera frame 
            # rvec(in rad) is the the rotaion vector of the board frame wrt camera frame, expressed in axis-angle representation
            if markNum != 0:
                # Replace tvec with 3D coordinates of  deprojected pixel corresponding to any point of the board
                # Here, we take the center of the marker of ID 0 or 3. Note that we take these
                # IDs because we assume that there will always be one of them detected 
                found_ID= False
                ##################CHANGE_IF_NEEDED##################
                id1=7
                id2=12
                ##################CHANGE_IF_NEEDED##################
                # look first for maker with ID= id1
                for j in range(len(ids)): 
                    if ids[j][0]==id1:
                        found_ID= True
                        break
                # If ID 0 not found, look for ID= id2    
                if found_ID==False:
                    for j in range(len(ids)): 
                        if ids[j][0]==id2:
                            break
                
                if not found_ID:
                    continue
                # Get pixel coordinates of the center of chosen marker 
                x=0 
                y=0
                for i in range(4):
                    x=x+corners[j][0][i][0]
                    y=y+corners[j][0][i][1]

                x= int(x/4)
                y= int(y/4)


                # Deprojection 
                depth= depth_frame.get_distance(x,y)
                deprojected_point = rs.rs2_deproject_pixel_to_point(color_intrinsics,[x,y], depth)
                # new tvec
                tvec=np.array(deprojected_point)*1000# in mm
            
                # Now, you have a board frame with the following transformation wrt camera frame 
                (R_board2cam, jac) = cv2.Rodrigues(rvec) # go to rotation matrix representation 
                ##################FOR_MICHELE#################
                # # Re-orient the axes by 180 deg rotation around x ( python compatibility issue)
                R_board2cam= np.dot(R_board2cam,Rotx(np.pi))
                ##################FOR_MICHELE#################
                (rvec, jac) = cv2.Rodrigues(R_board2cam)
                t_board2cam = tvec
                

                # Get roration matrix of block frame wrt camera frame 
                # Rotation around x by  90 degrees 
                alpha= (-np.pi/2)
                R_block2board= Rotx(alpha)
                R_block2cam=np.dot(R_board2cam,R_block2board)

                # Get rotation matrix of block frame wrt base 
                R_block2base= np.dot(R_cam2base,R_block2cam)
            

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
                t_base2crane = np.array([x,y,z])
                

                # Rotation around z by gamma
                R_block2crane= np.dot(R_base2crane,R_block2base)

                (rvec, jac) = cv2.Rodrigues(R_block2crane)
                

                # Now, we have the orientation of the block frame wrt crane  frame
                # we need to get the 3D coordiates of its center


                # We can transform the cog point in board frame to base frame 
                # The coordinates of desired po
                # Rotation around z by 43 degeesint in board frame(in mm) are: 
                id= ids[j][0]

                # This is the cog of the block 
                ##################CHANGE_IF_NEEDED##################
                if id==id1: 
                    pt_wrt_board= np.array([7 ,0,-60]) 
                if id==id2:
                    pt_wrt_board= np.array([7, 49.5+6.8+49.5 ,-60]) 
        
                ##################CHANGE_IF_NEEDED##################
    

                ## Transforming of cog from board frame to camera frame 

                # Rotation 
                rotated_point= np.dot(R_board2cam,pt_wrt_board)
                rotated_point= np.ravel(rotated_point, order='C') # make it one dimensional numpy.ndarray

                # Translation 
                a= t_board2cam[0]+rotated_point[0]
                b= t_board2cam[1]+rotated_point[1]
                c= t_board2cam[2]+rotated_point[2]
                pt_wrt_cam= [a,b,c]       

                ## Transforming of cog from camera frame to base frame 

                # Rotation 
                rotated_point= np.dot(R_cam2base,pt_wrt_cam)
                rotated_point= np.ravel(rotated_point, order='C') # make it one dimensional numpy.ndarray

                # Translation 
                a= t_cam2base[0]+rotated_point[0]
                b= t_cam2base[1]+rotated_point[1]
                c= t_cam2base[2]+rotated_point[2]
                pt_wrt_base= [a,b,c] 
                
                ## Transforming of cog from base frame to crane frame 

                # Rotation 
                rotated_point= np.dot(R_base2crane,pt_wrt_base)
                rotated_point= np.ravel(rotated_point, order='C') # make it one dimensional numpy.ndarray

                # Translation 
                a= t_base2crane[0]+rotated_point[0]
                b= t_base2crane[1]+rotated_point[1]
                c= t_base2crane[2]+rotated_point[2]
                pt_wrt_crane= [a,b,c] # this is the cog of the block 

                # Recording pose 
                ##################CHANGE_IF_NEEDED##################
                mov_avg_size= 0 # moving average filter window size 
                ##################CHANGE_IF_NEEDED##################
                
                # we need at least "mov_avg_size" poses   

                # previous pose
                # current pose 
                # moving average on tranlsation vector
                t1=  pt_wrt_crane[0]*0.001+0.006 #0.006 is an offset
                t2=  pt_wrt_crane[1]*0.001+0.008
                t3=  pt_wrt_crane[2]*0.001+0.0094 #0.0094 is an offset
                #print(t3+l+height)
                t_block2crane= np.array([t1,t2,t3])
                # moving average on rotation vector 
                #rvec= np.array([np.mean(pose[k-mov_avg_size:k+1,3]),np.mean(pose[k-mov_avg_size:k+1,4]),np.mean(pose[k-mov_avg_size:k+1,5])])
                # Rotation matrix elements 
                (R_block2crane, jac) = cv2.Rodrigues(rvec) 
                r11= R_block2crane[0][0]
                r12= R_block2crane[0][1]
                r13= R_block2crane[0][2]
                r21= R_block2crane[1][0]
                r22= R_block2crane[1][1]
                r23= R_block2crane[1][2]
                r31= R_block2crane[2][0]
                r32= R_block2crane[2][1]
                r33= R_block2crane[2][2]


                # Inverse kinematics 
                #q_curr = inv_kin(t_block2crane ,R_block2crane,l,height)
                #q_curr=np.random.normal(0,0.0001,5)
                #data.data= q_curr
                data.data= np.array([t1,t2,t3,r11,r12,r13,r21,r22,r23,r31,r32,r33])
                

                # sending data to KF node
                pub_x.publish(data) 

                # sending data to visual node
                data.data= np.array([t1,t2,t3])
                pub_res.publish(data)
                print("Time:"+ str(time.time()-start) + ' s')
                
        rate.sleep()
        #print("Frequency:"+ str(1/(time.time()-start)) + ' Hz')
       
        
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass