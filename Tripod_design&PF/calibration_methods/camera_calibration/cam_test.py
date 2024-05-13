#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import os
import sys
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber

script_path = os.path.dirname(os.path.abspath(__file__))
module_path = os.path.dirname(script_path)
if module_path not in sys.path:
    sys.path.append(module_path)
from core.StereoCamera import StereoCamera

camera_file = '/home/erie-dvrk/force_sensor_particle_filter/calibration_param/B3_cal.yaml'

def test_callback(image1: Image, image2: Image):
    
    try:
        im1 = np.ndarray(shape=(image1.height, image1.width, 3), 
                         dtype=np.uint8, buffer=image1.data)
        im2 = np.ndarray(shape=(image2.height, image2.width, 3), 
                         dtype=np.uint8, buffer=image2.data)
        
        im1, im2 = cam.processImage(im1, im2)
        
        cv2.imshow("left", im1)
        cv2.imshow("right", im2)
        cv2.waitKey(1)
        
    except Exception as e:
        rospy.logerr("Error converting images: {}".format(e))
        return
    
if __name__ == '__main__':
    rospy.init_node('test_node')
    cam = StereoCamera(camera_file, rectify=False, downscale_factor=1)
    
    l_image_sub = Subscriber('davinci_endo/left/image_raw', Image)
    r_image_sub = Subscriber('davinci_endo/right/image_raw', Image)
    
    ats = ApproximateTimeSynchronizer([l_image_sub, r_image_sub], 
                                      queue_size=5, slop=0.1)
    ats.registerCallback(test_callback)

    rospy.spin()
