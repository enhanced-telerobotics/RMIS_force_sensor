import rospy
import cv2
import time
import os
import sys

# from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, JointState
from force_sensor.msg import DoubleWrenchStamped, StampedWrench
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError

script_path = os.path.dirname(os.path.abspath(__file__))
module_path = os.path.dirname(script_path)
if module_path not in sys.path:
    sys.path.append(module_path)
# print(module_path)

from core.utils import *
from core.RobotLink import *
from core.StereoCamera import *
from core.ParticleFilter import *
from core.probability_functions import *

# File inputs
robot_file    = script_path + '/../cal_param/LND_sensor.json'
camera_file   = script_path + '/../cal_param/B3_cal.yaml'
hand_eye_file = script_path + '/../cal_param/handeye_cal.yaml'
# Output data
data_path = 'PF_test_data.txt'

# ROS Topics
force_topic = '/wrench'
standard_force_topic = '/atiSensor'
left_camera_topic  = '/davinci_endo/left/image_raw'
right_camera_topic = '/davinci_endo/right/image_raw'
robot_joint_topic  = '/PSM2/measured_js'
robot_gripper_topic = '/PSM2/jaw/measured_js'

# Initialize global variables for callback function
cam = None # cam is global so we can "processImage" in callback
cb_detected_keypoints_l = None
cb_detected_keypoints_r = None
cb_left_img = None
cb_right_img = None
cb_joint_angles = None
left_force = None
right_force = None
new_cb_data = False

# ROS Callback for images and joint observations
def gotData(l_img_msg:Image, r_img_msg:Image, j_msg:JointState, g_msg:JointState, 
            force_msg:DoubleWrenchStamped, ati_msg:StampedWrench):
    global cam, new_cb_data, cb_detected_keypoints_l, cb_detected_keypoints_r, cb_left_img, cb_right_img, cb_joint_angles
    global cb_left_force, cb_right_force, cb_ati_force
    try:
        # convert images to ndarrays
        _cb_left_img  = np.ndarray(shape=(l_img_msg.height, l_img_msg.width, 3),
                                      dtype=np.uint8, buffer=l_img_msg.data)
        _cb_right_img = np.ndarray(shape=(r_img_msg.height, r_img_msg.width, 3),
                                      dtype=np.uint8, buffer=r_img_msg.data)
        # resize and remap images
        _cb_left_img, _cb_right_img = cam.processImage(_cb_left_img, _cb_right_img)
    except:
        return
    
    # find painted point features on tool
    cb_detected_keypoints_l, _cb_left_img  = segmentColorAndGetKeyPoints(_cb_left_img, 
                                                                         hsv_min=(0, 120, 160), hsv_max=(50, 255, 255), 
                                                                         draw_contours=True)
    cb_detected_keypoints_r, _cb_right_img = segmentColorAndGetKeyPoints(_cb_right_img, 
                                                                         hsv_min=(0, 100, 100), hsv_max=(50, 255, 255), 
                                                                         draw_contours=True)

    cb_right_img = np.copy(_cb_right_img)
    cb_left_img  = np.copy(_cb_left_img)
    
    base_angle = 1.344 # offset of gripper angle (when gripper angle=base angle, angle between two sensors is 0)
    min_angle = 0 # smallest angle between sensors
    
    cb_joint_angles = np.array(j_msg.position + g_msg.position)
    cb_joint_angles[6] = cb_joint_angles[6] - base_angle
    if cb_joint_angles[6] <= min_angle:
        cb_joint_angles[6] = min_angle
    
    cb_left_force = vector3_to_arr(force_msg.left_wrench.force)
    cb_right_force = vector3_to_arr(force_msg.right_wrench.force)
    cb_ati_force = vector3_to_arr(ati_msg.wrench.force)

    new_cb_data = True

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('force_sensor_PF', anonymous=True)
    start_time = time.time()
    
    # Initialize transformed force publishers
    LF_base_pub = rospy.Publisher('/LF_base', Vector3, queue_size=10)
    RF_base_pub = rospy.Publisher('/RF_base', Vector3, queue_size=10)
    Fnet_pub = rospy.Publisher('/Fnet', Vector3, queue_size=10)
    Fgrip_pub = rospy.Publisher('/Fgrip', Float32, queue_size=10)
    ati_pub = rospy.Publisher('/ATI_base', Vector3, queue_size=10)
    left_image_pub = rospy.Publisher('/Left_image_PF', Image, queue_size=10)
    right_image_pub = rospy.Publisher('/Right_image_PF', Image, queue_size=10)
    
    # Initialize subscribers
    l_image_sub = Subscriber(left_camera_topic, Image)
    r_image_sub = Subscriber(right_camera_topic, Image)
    robot_j_sub = Subscriber(robot_joint_topic, JointState)
    gripper_j_sub = Subscriber(robot_gripper_topic, JointState)
    force_sensor_sub = Subscriber(force_topic, DoubleWrenchStamped)
    ati_sub = Subscriber(standard_force_topic, StampedWrench)
    
    # synchronize all topics
    ats = ApproximateTimeSynchronizer([l_image_sub, r_image_sub, robot_j_sub, gripper_j_sub, force_sensor_sub, ati_sub], 
                                      queue_size=5, slop=0.01) # slop = 0.015
    ats.registerCallback(gotData)
    
    # instantiate robot and camera
    robot_arm = RobotLink(robot_file)
    cam = StereoCamera(camera_file, rectify=False, downscale_factor=1)
    
    # load hand-eye transform
    f = open(hand_eye_file)
    hand_eye_data = yaml.load(f, Loader=yaml.FullLoader)
    # process hand-eye data
    cam_T_b = np.eye(4)
    cam_T_b[:-1, -1] = np.array(hand_eye_data['PSM1_tvec']) / 1000.0 # unit:mm    
    cam_T_b[:-1, :-1] = np.array(hand_eye_data['PSM1_rmat']).reshape(3,3)
    
    # get transformation from ATI sensor to PSM base
    ati_align= np.array(hand_eye_data['ATI_rmat']).reshape(3, 3)

    # Initialize filter
    pf = ParticleFilter(num_states=9, 
                        initialDistributionFunc=sampleNormalDistribution,
                        motionModelFunc=lumpedErrorMotionModel,
                        obsModelFunc=pointFeatureObs,
                        num_particles=200)

    init_kwargs = {
                    "std": np.array([1.0e-3, 1.0e-3, 1.0e-3, # pos
                                    1.0e-2, 1.0e-2, 1.0e-2, # ori
                                    0.0, 0.0, 0.0])   # joints
                  }

    pf.initializeFilter(**init_kwargs)
    
    rate = rospy.Rate(60)
    prev_joint_angles = None
    
    # Initialize force sensor variables
    F_net = np.zeros(3)
    F_grip = 0
    leftGripToSensor = np.array([1, 0, 0, 0, 0, 1, 0, -1, 0]).reshape(3, 3)
    rightGripToSensor = np.array([-1, 0, 0, 0, 0, 1, 0, 1, 0]).reshape(3, 3)
    
    with open(data_path, 'a') as file:
        file.write("Time\tATI.x\tATI.y\tATI.z\tF_net.x\tF_net.y\tF_net.z\tF_grip\n")
    
    rospy.loginfo("Particle filter initialized.")

    # Main loop
    while not rospy.is_shutdown():
        
        if new_cb_data:
            start_t = time.time()
            
            # Copy all the new data so they don't get over-written by callback
            new_detected_keypoints_l = np.copy(cb_detected_keypoints_l)
            new_detected_keypoints_r = np.copy(cb_detected_keypoints_r)
            new_left_img = np.copy(cb_left_img)
            new_right_img = np.copy(cb_right_img)
            new_joint_angles = np.copy(cb_joint_angles)
            left_force = np.copy(cb_left_force)
            right_force = np.copy(cb_right_force)
            ati_force = np.copy(cb_ati_force)
            new_cb_data = False
            
            # First time
            if prev_joint_angles is None:
                prev_joint_angles = new_joint_angles
            
            # Predict Particle Filter
            robot_arm.updateJointAngles(new_joint_angles)
            j_change = new_joint_angles - prev_joint_angles

            std_j = np.abs(j_change)*0.01
            std_j[-3:] = 0.0

            pred_kwargs = {
                            "std_pos": 2.5e-5, 
                            "std_ori": 1.0e-4,
                            "robot_arm": robot_arm, 
                            "std_j": std_j,
                            "nb": 4
                          }
            pf.predictionStep(**pred_kwargs) 
            
            # Update Particle Filter
            upd_kwargs = {
                            "point_detections": (new_detected_keypoints_l, new_detected_keypoints_r), 
                            "robot_arm": robot_arm, 
                            "cam": cam, 
                            "cam_T_b": cam_T_b,
                            "joint_angle_readings": new_joint_angles,
                            "gamma": 0.15
            }

            pf.updateStep(**upd_kwargs)
            prev_joint_angles = new_joint_angles

            correction_estimation = pf.getMeanParticle()
            
            T = poseToMatrix(correction_estimation[:6])
            new_joint_angles[-(correction_estimation.shape[0]-6):] += correction_estimation[6:]
            robot_arm.updateJointAngles(new_joint_angles)
            
            # Rotation matrices from base or wrist to left and right sensor
            T_base_left  = np.matmul(robot_arm.baseToJointT[-2][:3, :3], leftGripToSensor)
            T_base_right = np.matmul(robot_arm.baseToJointT[-1][:3, :3], rightGripToSensor)
            T_wrist_left  = np.matmul(np.matmul(np.linalg.inv(robot_arm.baseToJointT[-3][:3, :3]), 
                                                robot_arm.baseToJointT[-2][:3, :3]), leftGripToSensor)
            T_wrist_right = np.matmul(np.matmul(np.linalg.inv(robot_arm.baseToJointT[-3][:3, :3]), 
                                                robot_arm.baseToJointT[-1][:3, :3]), rightGripToSensor)
            
            # Left and right forces in base or wrist frames
            LF_base = np.dot(T_base_left, left_force)
            RF_base = np.dot(T_base_right, right_force)
            LF_wrist = np.dot(T_wrist_left, left_force)
            RF_wrist = np.dot(T_wrist_right, right_force)
            
            F_net = LF_base + RF_base
            F_grip = min(np.abs(LF_wrist[0]), np.abs(RF_wrist[0]))
            ati_base = np.dot(ati_align, ati_force)
            
            # rospy.loginfo("Time to predict & update {}".format(time.time() - start_t))
            elapsed_time = time.time() - start_time
            
            # convert forces to vectors for publishing
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
            
            # Project skeleton
            img_list = projectSkeleton(robot_arm.getSkeletonPoints(), np.dot(cam_T_b, T),
                                       [new_left_img, new_right_img], cam.projectPoints)
            
            # display images with point features and skeleton
            cv2.namedWindow("Left Img", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Right Img", cv2.WINDOW_NORMAL)
            
            cv2.resizeWindow("Left Img", 1920, 1200)
            cv2.resizeWindow("Right Img", 1920, 1200)
            
            cv2.moveWindow("Left Img", 100, 150)
            cv2.moveWindow("Right Img", 2660, 150)
            
            cv2.imshow("Left Img", img_list[0][:,:,[0,1,2]])
            cv2.imshow("Right Img", img_list[1][:,:,[0,1,2]])
            cv2.waitKey(1)
            
            bridge = CvBridge()
            left_image = bridge.cv2_to_imgmsg(img_list[0][:,:,[0,1,2]], "bgr8")
            right_image = bridge.cv2_to_imgmsg(img_list[1][:,:,[0,1,2]], "bgr8")
            left_image_pub.publish(left_image)
            right_image_pub.publish(right_image)
            
        rate.sleep()