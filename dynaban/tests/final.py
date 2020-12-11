#!/usr/bin/python

################################################################################

# Final code for imitating the human actions recorded by Kinect sensor

################################################################################


import roslib
roslib.load_manifest('skeleton')
import rospy
import tf
import time
from tf.transformations import *
import os
from math import *
import numpy as np
from write_servo.py import *


v = [1,0,0,0]

ltheta2 = rtheta2 = 0
ltheta2_X = ltheta2_Y = ltheta2_Z = 0
rtheta2_X = rtheta2_Y = rtheta2_Z = 0

if __name__ == '__main__':

    rospy.init_node('kinect_listener',anonymous=True)
    listener = tf.TransformListener()
    
    enable_port()                                            # Opening the port and communicating with Dynamixel Controller

    enable_bot(1)                                            # Enabling torques of various servos
    enable_bot(2)
    enable_bot(3)
    enable_bot(4)
    enable_bot(5)
    enable_bot(6)

    while not rospy.is_shutdown():
         try:
             transr, rotr = listener.lookupTransform('/openni_depth_frame', '/right_shoulder_1',rospy.Duration(0))    # Receiving Quaternions of right shoulder from Kinect sensor
             rot_conjr =  quaternion_conjugate(rotr)
             nr = quaternion_multiply(rotr,v)
             mr = quaternion_multiply(nr,rot_conjr)
             transr, rotr = listener.lookupTransform('/openni_depth_frame', '/right_elbow_1',rospy.Duration(0))       # Receiveing Quaternions of right elbow
             rot_conjr =  quaternion_conjugate(rotr)
             pr = quaternion_multiply(rotr,v)
             qr = quaternion_multiply(pr,rot_conjr)
             mr = mr*30                                                                                               # Scaling the human shoulder to elbow
             qr = qr*30                                                                                               # Scaling the human elbow to wrist
	     
             if (abs(mr[0]) > 26):
                 if (mr[0] < 0):
                     mr[0] = -30
                     mr[1] = mr[2] = 0
                 else:
                     mr[0] = 30
                     mr[1] = mr[2] = 0
             elif (abs(mr[1]) > 26):
                 if (mr[1] < 0):
                     mr[1] = -30
                     mr[0] = mr[2] = 0
                 else:
                     mr[1] = 30
                     mr[0] = mr[2] = 0

             elif (abs(mr[2]) > 26):
                 if (mr[2] < 0):
                     mr[0] = mr[1] = 0
                     mr[2] = -30
                 else:
                     mr[0] = mr[1] = 0
                     mr[2] = 30

             elif (abs(mr[0]) > 8 and abs(mr[0]) < 26 and abs(mr[1]) > 8 and abs(mr[1]) < 26 and abs(mr[2]) > 8 and abs(mr[2]) < 26 ):
                 if (mr[0] < 0):
                     mr[0] = -17
                 else:
                     mr[0] = 17
                 if (mr[1] < 0):
		    		 mr[1] = -17
                 else:
		    		 mr[1] = 17
                 if (mr[2] < 0):
		    		 mr[2] = -17
                 else:
                     mr[2] = 17

             elif (abs(mr[0]) < 9):
                 if (abs(mr[1]) > 15 and abs(mr[1]) < 26 and abs(mr[2]) > 15 and abs(mr[2]) < 26):
                     if (mr[1] < 0):
                         mr[1] = -21
                         mr[0] = 0
                     else:
                         mr[1] = 21
                         mr[0] = 0
                     if (mr[2] < 0):
                         mr[2] = -21
                         mr[0] = 0
                     else:
                         mr[2] = 21
                         mr[0] = 0

             elif (abs(mr[1]) < 9):
                 if (abs(mr[0]) > 15 and abs(mr[0]) < 26 and abs(mr[2]) > 15 and abs(mr[2]) < 26):
                     if (mr[0] < 0):
                         mr[0] = -21
                         mr[1] = 0
                     else:
                         mr[0] = 21
                         mr[1] = 0
                     if (mr[2] < 0):
                         mr[2] = -21
                         mr[1] = 0
                     else:
                         mr[2] = 21
                         mr[1] = 0

             elif (abs(mr[2]) < 9):
                 if (abs(mr[0]) > 15 and abs(mr[0]) < 26 and abs(mr[1]) > 15 and abs(mr[1]) < 26):
                     if (mr[0] < 0):
                         mr[0] = -21
                         mr[2] = 0
                     else:
                         mr[0] = 21
                         mr[2] = 0
                     if (mr[1] < 0):
                         mr[1] = -21
                         mr[2] = 0
                     else:
                         mr[1] = 21
                         mr[2] = 0


             if (abs(qr[0]) > 24):
                     if (qr[0] < 0):
                         qr[0] = -30
                         qr[1] = qr[2] = 0
                     else:
                         qr[0] = 30
                         qr[1] = qr[2] = 0
             elif (abs(qr[1]) > 24):
                     if (qr[1] < 0):
                         qr[1] = -30
                         qr[0] = qr[2] = 0
                     else:
                         qr[1] = 30
                         qr[0] = qr[2] = 0
             elif (abs(qr[2]) > 24):
                     if (qr[2] < 0):
                         qr[0] = qr[1] = 0
                         qr[2] = -30
                     else:
                         qr[0] = qr[1] = 0
                         qr[2] = 30


             elif (abs(qr[0]) > 8 and abs(qr[0]) < 24 and abs(qr[1]) > 8 and abs(qr[1]) < 24 and abs(qr[2]) > 8 and abs(qr[2]) < 24 ):
                 if (qr[0] < 0):
                     qr[0] = -17
                 else:
                     qr[0] = 17
                 if (qr[1] < 0):
                     qr[1] = -17
                 else:
                     qr[1] = 17
                 if (qr[2] < 0):
                     qr[2] = -17
                 else:
                     qr[2] = 17

             elif (abs(qr[0]) < 9):
                 if (abs(qr[1]) > 15 and abs(qr[1]) < 24 and abs(qr[2]) > 15 and abs(qr[2]) < 24):
                     if (qr[1] < 0):
                         qr[1] = -21
                         qr[0] = 0
                     else:
                         qr[1] = 21
                         qr[0] = 0
                     if (qr[2] < 0):
                         qr[2] = -21
                         qr[0] = 0
                     else:
                         qr[2] = 21
                         qr[0] = 0

             elif (abs(qr[1]) < 9):
                 if (abs(qr[0]) > 15 and abs(qr[0]) < 24 and abs(qr[2]) > 15 and abs(qr[2]) < 24):
                     if (qr[0] < 0):
                         qr[0] = -21
                         qr[1] = 0
                     else:
                         qr[0] = 21
                         qr[1] = 0
                     if (qr[2] < 0):
                         qr[2] = -21
                         qr[1] = 0
                     else:
                         qr[2] = 21
                         qr[1] = 0

             elif (abs(qr[2])<9):
                 if (abs(qr[0])>15 and abs(qr[0])<24 and abs(qr[1])>15 and abs(qr[1])<24):
                     if (qr[0] < 0):
                         qr[0] = -21
                         qr[2] = 0
                     else:
                         qr[0] = 21
                         qr[2] = 0
                     if (qr[1] < 0):
                         qr[1] = -21
                         qr[2] = 0
                     else:
                         qr[1] = 21
                         qr[2] = 0

             posr = mr+qr                                                    # Resultant vector from the right shoulder to end effector tool

             q_right = [0.5,0.5,0.5,0.5]
             q_right_conj = [-0.5,-0.5,-0.5,0.5]
             vect = [posr[0],posr[1],posr[2],0]

             calc = quaternion_multiply(q_right,vect)
             vector = quaternion_multiply(calc,q_right_conj)

             xr = vector[0]
             yr = vector[1]
             zr = vector[2]

             xr_elbow = mr[2]
             yr_elbow = mr[0]
             zr_elbow = mr[1]
          
             int_theta = np.array([0,0,0],dtype=np.float64)
             xyzr = (xr,yr,zr)
          
             if yr != 0 and mr[0] == mr[2] == 0 and mr[1] == 30 :
                 rtheta0_elbow = -pi/2
             elif yr_elbow == 0 and xr_elbow == 0 and yr == 0 :
                 rtheta0_elbow = 0
             else:
                 rtheta0_elbow = -atan2((yr_elbow),(xr_elbow))
            
             rtheta1_elbow = -asin(zr_elbow/30)

             try:
                 rtheta2_Z = -asin((zr+30*sin(rtheta1_elbow))/30)-rtheta1_elbow
             except  :
                 print('error in zr')
                 pass
             try:
                 rtheta2_Y = acos((yr-30*sin(rtheta0_elbow)*cos(rtheta1_elbow))/(30*sin(rtheta0_elbow))) - rtheta1_elbow
             except  :
                 print('error in yr')
                 pass
             try:
                 rx = (xr-30*cos(rtheta0_elbow)*cos(rtheta1_elbow))/(30*cos(rtheta0_elbow))
                 rx = float('%.5f' %(rx))
                 rtheta2_X = acos(rx) - rtheta1_elbow
             except  :
                 print('error in xr')
                 pass

############################################## RIGHT HAND ########################################################################

             transl, rotl = listener.lookupTransform('/openni_depth_frame', '/left_shoulder_1',rospy.Duration(0))
             rot_conjl =  quaternion_conjugate(rotl)
             nl = quaternion_multiply(rotl,v)
             ml = quaternion_multiply(nl,rot_conjl)
             transl, rotl = listener.lookupTransform('/openni_depth_frame', '/left_elbow_1',rospy.Duration(0))
             rot_conjl =  quaternion_conjugate(rotl)
             pl = quaternion_multiply(rotl,v)
             ql = quaternion_multiply(pl,rot_conjl)

             ml = ml*30
             ql = ql*30

             if (abs(ml[0]) > 26):
                 if (ml[0] < 0):
                     ml[0] = 30
                     ml[1] = ml[2] = 0
                 else:
                     ml[0] = -30
                     ml[1] = ml[2] = 0

             elif (abs(ml[1]) > 26):
                 if (ml[1] < 0):
                     ml[1] = -30
                     ml[0] = ml[2] = 0
                 else:
                     ml[1] = 30
                     ml[0] = ml[2] = 0

             elif (abs(ml[2]) > 26):
                 if (ml[2] < 0):
                     ml[0] = ml[1] = 0
                     ml[2] = 30
                 else:
                     ml[0] = ml[1] = 0
                     ml[2] = -30


             elif (abs(ml[0]) > 8 and abs( ml[0]) < 26 and abs(ml[1]) > 8 and abs(ml[1]) < 26 and abs(ml[2]) > 8 and abs(ml[2]) < 26 ):
                 if (ml[0] < 0):
                     ml[0] = 17
                 else:
                     ml[0] = -17
                 if (ml[1] < 0):
                     ml[1] = -17
                 else:
                     ml[1] = 17
                 if (ml[2] < 0):
                     ml[2] = 17
                 else:
                     ml[2] = -17


             elif (abs(ml[0]) < 9):
                 if (abs(ml[1]) > 15 and abs(ml[1]) < 26 and abs(ml[2]) > 15 and abs(ml[2]) < 26):
                     if (ml[1] < 0):
                         ml[1] = -21
                         ml[0] = 0
                     else:
                         ml[1] = 21
                         ml[0] = 0
                     if (ml[2] < 0):
                         ml[2] = 21
                         ml[0] = 0
                     else:
                         ml[2] = -21
                         ml[0] = 0

             elif (abs(ml[1]) < 9):
                 if (abs(ml[0]) > 15 and abs(ml[0]) < 26 and abs(ml[2]) > 15 and abs(ml[2]) < 26):
                     if (ml[0] < 0):
                         ml[0] = 21
                         ml[1] = 0
                     else:
                         ml[0] = -21
                         ml[1] = 0
                     if (ml[2] < 0):
                         ml[2] = 21
                         ml[1] = 0
                     else:
                         ml[2] = -21
                         ml[1] = 0

             elif (abs(ml[2])<9):
                 if (abs(ml[0])>15 and abs(ml[0])<26 and abs(ml[1])>15 and abs(ml[1])<26):
                     if (ml[0] < 0):
                         ml[0] = 21
                         ml[2] = 0
                     else:
                         ml[0] = -21
                         ml[2] = 0
                     if (ml[1] < 0):
                         ml[1] = -21
                         ml[2] = 0
                     else:
                         ml[1] = 21
                         ml[2] = 0

             if (abs(ql[0]) > 24):
                 if (ql[0] < 0):
                     ql[0] = 30
                     ql[1] = ql[2] = 0
                 else:
                     ql[0] = -30
                     ql[1] = ql[2] = 0

             elif (abs(ql[1]) > 24):
                 if (ql[1] < 0):
                     ql[1] = -30
                     ql[0] = ql[2] = 0
                 else:
                     ql[1] = 30
                     ql[0] = ql[2] = 0

             elif (abs(ql[2]) > 24):
                 if (ql[2] < 0):
                     ql[0] = ql[1] = 0
                     ql[2] = 30
                 else:
                     ql[0] = ql[1] = 0
                     ql[2] = -30


             elif (abs(ql[0]) > 8 and abs(ql[0]) < 24 and abs(ql[1]) > 8 and abs(ql[1]) < 24 and abs(ql[2]) > 8 and abs(ql[2]) < 24 ):
                 if (ql[0] < 0):
                     ql[0] = 17
                 else:
                     ql[0] = -17
                 if (ql[1] < 0):
                     ql[1] = -17
                 else:
                     ql[1] = 17
                 if (ql[2] < 0):
                     ql[2] = 17
                 else:
                     ql[2] = -17

             elif (abs(ql[0]) < 9):
                 if (abs(ql[1]) > 15 and abs(ql[1]) < 24 and abs(ql[2]) > 15 and abs(ql[2]) < 24):
                     if (ql[1] < 0):
                         ql[1] = -21
                         ql[0] = 0
                     else:
                         ql[1] = 21
                         ql[0] = 0
                     if (ql[2] < 0):
                         ql[2] = 21
                         ql[0] = 0
                     else:
                         ql[2] = -21
                         ql[0] = 0

             elif (abs(ql[1]) < 9):
                 if (abs(ql[0]) > 15 and abs(ql[0]) < 24 and abs(ql[2]) > 15 and abs(ql[2]) < 24):
                     if (ql[0] < 0):
                         ql[0] = 21
                         ql[1] = 0
                     else:
                         ql[0] = -21
                         ql[1] = 0
                     if (ql[2] < 0):
                         ql[2] = 21
                         ql[1] = 0
                     else:
                         ql[2] = -21
                         ql[1] = 0

             elif (abs(ql[2])<9):
                 if (abs(ql[0])>15 and abs(ql[0])<24 and abs(ql[1])>15 and abs(ql[1])<24):
                     if (ql[0] < 0):
                         ql[0] = 21
                         ql[2] = 0
                     else:
                         ql[0] = -21
                         ql[2] = 0
                     if (ql[1] < 0):
                         ql[1] = -21
                         ql[2] = 0
                     else:
                         ql[1] = 21
                         ql[2] = 0

             posl = ml+ql

             l_right = [0.5,0.5,0.5,0.5]
             l_right_conj = [-0.5,-0.5,-0.5,0.5]
             vect = [posl[0],posl[1],posl[2],0]

             calc = quaternion_multiply(l_right,vect)
             vector_right = quaternion_multiply(calc,l_right_conj)

             xl = vector_right[0]
             yl = vector_right[1]
             zl = vector_right[2]
            
             int_theta = np.array([0,0,0],dtype=np.float64)
             xyzl = (xl,yl,zl)
           
             xl_elbow = ml[2]
             yl_elbow = ml[0]
             zl_elbow = ml[1]

             #print(yl,yl_elbow,xl_elbow,zl_elbow)
             if yl != 0 and yl_elbow == xl_elbow == 0 and zl_elbow == 30 :
                 ltheta0_elbow = -pi/2
                 print('testing')
             elif yl_elbow == 0 and xl_elbow == 0 and yl == 0 :
                 ltheta0_elbow = 0
             else:
                 ltheta0_elbow = -atan2((yl_elbow),(xl_elbow))

             ltheta1_elbow = -asin(zl_elbow/30)

             try:
                 ltheta2_Z = -asin((zl+30*sin(ltheta1_elbow))/30)-ltheta1_elbow
             except  :
                 print('error in zl')
                 pass
             try:
                 ltheta2_Y = acos((yl-30*sin(ltheta0_elbow)*cos(ltheta1_elbow))/(30*sin(ltheta0_elbow))) - ltheta1_elbow
             except  :
                 print('error in yl')
                 pass
             try:
                 xa = (xl-30*cos(ltheta0_elbow)*cos(ltheta1_elbow))/(30*cos(ltheta0_elbow))
                 xa = float('%.5f' %(xa))
                 ltheta2_X = acos(xa) - ltheta1_elbow
             except  :
                 print('error in xl')
                 pass

             ar = (mr[0]*qr[0]+mr[1]*qr[1]+mr[2]*qr[2])/900
             al = (ml[0]*ql[0]+ml[1]*ql[1]+ml[2]*ql[2])/900
             a_elbow_left = acos(al)
             a_elbow_right = acos(ar)

             if rtheta2_X > pi:
                 rtheta2_X = rtheta2_X - 2*pi
             if ltheta2_X > pi:
                 ltheta2_X = ltheta2_X - 2*pi

             if abs(abs(a_elbow_left) - abs(ltheta2_X)) < 5*pi/180 :
                ltheta2 = ltheta2_X
             else:
                 if abs(abs(a_elbow_left) - abs(ltheta2_Y)) < 5*pi/180 :
                     ltheta2 = ltheta2_Y
                 else:
                     if abs(abs(a_elbow_left) - abs(ltheta2_Z)) < 5*pi/180 :
                         ltheta2 = ltheta2_Z
             if abs(abs(a_elbow_right) - abs(rtheta2_X)) < 5*pi/180 :
                rtheta2 = rtheta2_X
             else:
                 if abs(abs(a_elbow_right) - abs(rtheta2_Y)) < 5*pi/180 :
                    rtheta2 = rtheta2_Y
                 else:
                     if abs(abs(a_elbow_right) - abs(rtheta2_Z)) < 5*pi/180 :
                         rtheta2 = rtheta2_Z
             if xr == zr == 30 and yr == 0 and rtheta1_elbow == 0:
                 rtheta2 = rtheta2_Z
             if xl == zl == 30 and yl == 0 and ltheta1_elbow == 0:
                 ltheta2 = ltheta2_Z

             ltheta2_X = ltheta2_Y = ltheta2_Z = 0
             rtheta2_X = rtheta2_Y = rtheta2_Z = 0

             lvalue0 = int(820+rtheta0_elbow*(180/pi)*1024/360)
             lvalue1 = int(740+rtheta1_elbow*(180/pi)*1024/360)
             if rtheta2 > pi :
                 lvalue2 = int(530+(rtheta2-2*pi)*(180/pi)*1024/360)
             else:
                 lvalue2 = int(530+(rtheta2)*(180/pi)*1024/360)

             rvalue0 = int(145-ltheta0_elbow*(180/pi)*1024/360)
             rvalue1 = int(270-ltheta1_elbow*(180/pi)*1024/360)
             rvalue2 = int(520-ltheta2*(180/pi)*1024/360)

             print(ltheta2)
             print(rtheta2)
             print(xl,yl,zl)
             print(xr,yr,zr)
             write_value(2, lvalue0)
             write_value(4, lvalue1)
             write_value(6, lvalue2)
             write_value(1, rvalue0)
             write_value(3, rvalue1)
             write_value(5, rvalue2)

         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
             continue
