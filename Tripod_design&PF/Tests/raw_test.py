#!/usr/bin/env python3
import rospy
import time
import os
import sys
import yaml
import numpy as np
from std_msgs.msg import Float32
from message_filters import ApproximateTimeSynchronizer, Subscriber
from force_sensor.msg import StampedWrench, DoubleWrenchStamped
from sensor_msgs.msg import JointState

script_path = os.path.dirname(os.path.abspath(__file__))
module_path = os.path.dirname(script_path)
if module_path not in sys.path:
    sys.path.append(module_path)

from core.utils import *
from core.RobotLink import *

robot_file    = script_path + '/../cal_param/LND_sensor.json'
hand_eye_file = script_path + '/../cal_param/handeye_cal.yaml'
data_path = 'raw_test_data.txt'

force_topic = '/wrench'
standard_topic = 'atiSensor'
robot_joint_topic  = '/PSM2/measured_js'
robot_gripper_topic = '/PSM2/jaw/measured_js'

cb_left_force = None
cb_right_force = None
received_data = False

def callback(ati_sensor_msg:StampedWrench, double_wrench_msg:DoubleWrenchStamped, joint_msg:JointState, gripper_msg:JointState):
    global cb_standard_force, cb_left_force, cb_right_force, cb_joint_angles, received_data
    
    cb_standard_force = vector3_to_arr(ati_sensor_msg.wrench.force)
    cb_left_force  = vector3_to_arr(double_wrench_msg.left_wrench.force)
    cb_right_force = vector3_to_arr(double_wrench_msg.right_wrench.force)
    
    base_angle = 1.3444
    min_angle = 0 
    
    cb_joint_angles = np.array(joint_msg.position + gripper_msg.position)
    cb_joint_angles[6] = cb_joint_angles[6] - base_angle
    if cb_joint_angles[6] <= min_angle:
        cb_joint_angles[6] = min_angle
    
    received_data = True
        
if __name__ == '__main__':
    rospy.init_node('data_collector', anonymous=True)
    start_time = time.time()
    
    LF_base_pub = rospy.Publisher('/LF_base', Vector3, queue_size=10)
    RF_base_pub = rospy.Publisher('/RF_base', Vector3, queue_size=10)
    Fnet_pub = rospy.Publisher('/Fnet', Vector3, queue_size=10)
    Fgrip_pub = rospy.Publisher('/Fgrip', Float32, queue_size=10)
    ati_pub = rospy.Publisher('/ATI_base', Vector3, queue_size=10)

    ati_sensor_sub = Subscriber(standard_topic, StampedWrench)
    double_wrench_sub = Subscriber(force_topic, DoubleWrenchStamped)
    joint_state_sub = Subscriber(robot_joint_topic, JointState)
    gripper_state_sub = Subscriber(robot_gripper_topic, JointState)

    ts = ApproximateTimeSynchronizer([ati_sensor_sub, double_wrench_sub, joint_state_sub, gripper_state_sub], 
                                     queue_size=10, slop=0.01)
    ts.registerCallback(callback)
    
    f = open(hand_eye_file)
    hand_eye_data = yaml.load(f, Loader=yaml.FullLoader)
    ati_align = np.array(hand_eye_data['ATI_rmat']).reshape(3, 3)
    
    robot_arm = RobotLink(robot_file)
    rate = rospy.Rate(60)
    F_net = np.zeros(3)
    F_grip = 0
    leftGripToSensor = np.array([1, 0, 0, 0, 0, 1, 0, -1, 0]).reshape(3, 3)
    rightGripToSensor = np.array([-1, 0, 0, 0, 0, 1, 0, 1, 0]).reshape(3, 3)
    leftGripToSensor_test = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
    rightGripToSensor_test = np.array([[-1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    
    with open(data_path, 'a') as file:
        file.write("Time\tATI.x\tATI.y\tATI.z\tF_net.x\tF_net.y\tF_net.z\tF_grip\n")
    
    rospy.loginfo("Recording force data.")
    
    while not rospy.is_shutdown():
        if received_data:
            
            ati_force = np.copy(cb_standard_force)
            left_force  = np.copy(cb_left_force)
            right_force = np.copy(cb_right_force)
            joint_angles = np.copy(cb_joint_angles)
            received_data = False
            robot_arm.updateJointAngles(joint_angles)
            
            T_base_left  = np.matmul(robot_arm.baseToJointT[-2][:3, :3], leftGripToSensor)
            T_base_right = np.matmul(robot_arm.baseToJointT[-1][:3, :3], rightGripToSensor)
            T_wrist_left  = np.matmul(np.matmul(np.linalg.inv(robot_arm.baseToJointT[-3][:3, :3]), 
                                                robot_arm.baseToJointT[-2][:3, :3]), leftGripToSensor)
            T_wrist_right = np.matmul(np.matmul(np.linalg.inv(robot_arm.baseToJointT[-3][:3, :3]), 
                                                robot_arm.baseToJointT[-1][:3, :3]), rightGripToSensor)
            
            LF_base = np.dot(T_base_left, left_force)
            RF_base = np.dot(T_base_right, right_force)
            LF_wrist = np.dot(T_wrist_left, left_force)
            RF_wrist = np.dot(T_wrist_right, right_force)
            
            F_net = LF_base + RF_base
            F_grip = min(np.abs(LF_wrist[0]), np.abs(RF_wrist[0]))
            ati_base = np.dot(ati_align, ati_force)
            
            elapsed_time = time.time() - start_time
            
            LF_vec = arr_to_vector3(LF_base)
            RF_vec = arr_to_vector3(RF_base)
            Fnet_vec = arr_to_vector3(F_net)
            ati_vec = arr_to_vector3(ati_base)
            
            LF_base_pub.publish(LF_vec)
            RF_base_pub.publish(RF_vec)
            Fnet_pub.publish(Fnet_vec)
            Fgrip_pub.publish(F_grip)
            ati_pub.publish(ati_vec)
            
            # write data into a text file
            with open(data_path, 'a') as file:
                file.write(f"{elapsed_time:.2f}\t"
                           f"{ati_base[0]:.2f}\t{ati_base[1]:.2f}\t{ati_base[2]:.2f}\t"
                           f"{F_net[0]:.2f}\t{F_net[1]:.2f}\t{F_net[2]:.2f}\t"
                           f"{F_grip:.2f}\n")
            
        rate.sleep()