#!/usr/bin/env python

'''
This node corrects the reported pose and then resolves the measured forces from the force sensor into the robot base frame
It is for dVRK 1.7.
Zonghe Chua
'''

import rospy
import dvrk
import PyKDL as kdl
import numpy as np
import scipy.interpolate as interp
from geometry_msgs.msg import Pose, Twist, Wrench
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String, Float64 ,Float64MultiArray
import os
import copy
import sys
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
import tf_conversions.posemath as pm

def quat_conj(quat):

    quat_conj = np.array([-quat[0],-quat[1],-quat[2],quat[3]])
    
    return quat_conj

def compute_jaws_pose(pose,jaw_pos, output='matrix', adjusted = False):
    
    # we need to derive the individual positions for the jaws based on the gripper yaw angle --> use the coupling matrix
    # but we need to threshold it because the dVRK reports the gripper closing even more than it actually does. This is done by setting the min_angle
    
    th = jaw_pos/2
    base_angle = 32.82 # the fixed angle to correct th 
    min_angle = 8.4/2 # SET THIS VALUE !! divide your desired minimum gripper angle by 2
    
    if adjusted:
        adjustment_angle = (base_angle+min_angle)*np.pi/180.0 this will create a minimum angle to report even if the dVRK reports an even smaller angle.
        if th < adjustment_angle:
            th = adjustment_angle
    
    H_wrist_6 = kdl.Frame(kdl.Rotation.RotX(th-base_angle))
    H_wrist_7 = kdl.Frame(kdl.Rotation.RotX(-th+base_angle))
    
    H6 = pose*H_wrist_6 # H_base_wrist * H_wrist_6 = H_base_6
    H7 = pose*H_wrist_7
    
    if output == 'pose':
        H6 = pm.toMsg(H6)
        H7 = pm.toMsg(H7)
        H_wrist_6 = pm.toMsg(H_wrist_6)
        H_wrist_7 = pm.toMsg(H_wrist_7)
    elif output =='matrix':
        H6 = pm.toMatrix(H6)
        H7 = pm.toMatrix(H7)
        H_wrist_6 = pm.toMatrix(H_wrist_6)
        H_wrist_7 = pm.toMatrix(H_wrist_7)
    
    return H6,H7,H_wrist_6,H_wrist_7
    
def angle_btw_quats(q1,q2):
    #find the angle difference between the two quaternions
    z = q1.conjugate*q2
    a = 2*np.arccos(z[0])
    
    return a

def leftWrench_cb(wrench):
    global leftWrench
    leftWrench[0] = wrench.force.x
    leftWrench[1] = wrench.force.y
    leftWrench[2] = wrench.force.z
    
def rightWrench_cb(wrench):
    global rightWrench
    rightWrench[0] = wrench.force.x
    rightWrench[1] = wrench.force.y
    rightWrench[2] = wrench.force.z

'''
-----------------------------------------------------------------------------
BEGIN SCRIPT
-----------------------------------------------------------------------------
'''

if __name__ == "__main__":    
    
    global leftWrench
    global rightWrench
    
    leftWrench = np.zeros(3)
    rightWrench = np.zeros(3)
    Fg = np.zeros(3)
    zp90 = np.array([[0,-1,0],[1,0,0],[0,0,1]]) # fixed transforms to rotate frames correctly
    zm90 = np.array([[0,1,0],[-1,0,0],[0,0,1]]) 
    
    out_form = 'matrix'
    psm = dvrk.psm('PSM2')
    
    # declare subscribers for the force topics reported from the Arduino
    rospy.Subscriber('leftWrench',Wrench,leftWrench_cb)
    rospy.Subscriber('rightWrench',Wrench,rightWrench_cb)
    result_pub = rospy.Publisher('resultWrench',Wrench,queue_size=1)
    grip_pub = rospy.Publisher('gripForce',Wrench,queue_size=1)
    
    resultWrench = Wrench()
    gripWrench = Wrench() #even though it is a scalar, just use a wrench to report it. Here we only use the z component.
    
    current_time = rospy.get_time()
    previous_time = rospy.get_time()
    
    
    while not rospy.is_shutdown():

        current_time = rospy.get_time()
        
        if (current_time - previous_time > 0.001):
            jaw_pos = psm.get_current_jaw_position()
            pose = psm.get_current_position()
            
            # compute the jaw pose
            LFrame,RFrame,LFrame_wrist,RFrame_wrist = compute_jaws_pose(pose,jaw_pos, output=out_form, adjusted = True)
            
            # resolve the forces into the base frame
            leftWrench_base = np.dot(np.matmul(LFrame[:3,:3],zp90),leftWrench)
            rightWrench_base = np.dot(np.matmul(RFrame[:3,:3],zm90),rightWrench)
            
            # resolve the resultant forces
            Fr = leftWrench_base+rightWrench_base
            
            resultWrench.force.x = Fr[0]
            resultWrench.force.y = Fr[1]
            resultWrench.force.z = -Fr[2]
            
            # get the forces in the wrist frame
            leftWrench_wrist = np.dot(np.matmul(LFrame_wrist[:3,:3],zp90),leftWrench)
            rightWrench_wrist = np.dot(np.matmul(RFrame_wrist[:3,:3],zm90),rightWrench)
            
            # the grip force is the minimum of the y_direction forces in the wrist frame
            if np.abs(leftWrench_wrist[1]) < np.abs(rightWrench_wrist[1]):
                Fg[2] = leftWrench_wrist[1]
            else:
                Fg[2] = -rightWrench_wrist[1]
            
            gripWrench.force.x = 0
            gripWrench.force.y = 0
            gripWrench.force.z = Fg[2]
            
            result_pub.publish(resultWrench)
            grip_pub.publish(gripWrench)
            
            
            previous_time = current_time
        
        
