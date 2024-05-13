import numpy as np
import cv2
import imutils
from geometry_msgs.msg import Vector3

def projectSkeleton(skeletonPts3D, cam_T_b, img_list, project_point_function):
    # skeletonPts3D should be in the same format as getSkeletonPoints from RobotLink
    # img_list
    for skeletonPairs in skeletonPts3D:
        pt_pair = np.transpose(np.array(skeletonPairs))
        pt_pair = np.concatenate((pt_pair, np.ones((1, pt_pair.shape[1]))))
        pt_pair = np.transpose(np.dot(cam_T_b, pt_pair)[:-1,:])
        proj_pts_list = project_point_function(pt_pair)

        # print(proj_pts_list[0])
        # print(proj_pts_list[1])
        # print()
        
        if len(img_list) != len(proj_pts_list):
            raise ValueError("Number of imgs inputted must equal the project_point_function cameras")
        
        for idx, proj_pts in enumerate(proj_pts_list):
            try:
                img_list[idx] = cv2.line(img_list[idx], (int(proj_pts[0,0]), int(proj_pts[0,1])), 
                                   (int(proj_pts[1,0]), int(proj_pts[1,1])),  (0,255,0), 5)
            except:
                continue

    return img_list

def vector3_to_arr(vec):
    return np.array([vec.x, vec.y, vec.z])

def arr_to_vector3(arr):
    return Vector3(x=arr[0], y=arr[1], z=arr[2])

def axisAngleToRotationMatrix(axis_angle):
    angle = np.linalg.norm(axis_angle)
    
    if angle < 1e-5:
        return np.eye(3)
    
    axis  = axis_angle/angle
    cross_product_mat_axis = np.array([[0, -axis[2], axis[1]],
                                       [axis[2], 0, -axis[0]],
                                       [-axis[1], axis[0], 0]])
    return np.cos(angle)*np.eye(3) + np.sin(angle) * cross_product_mat_axis \
            + (1.0 - np.cos(angle))*np.outer(axis,axis)

def rotationMatrixToAxisAngle(rotation_matrix):
    angle = np.arccos((rotation_matrix[0,0] + rotation_matrix[1,1] + rotation_matrix[2,2] - 1.0)/2.0 )
    axis  = np.array([rotation_matrix[2,1] - rotation_matrix[1,2], 
                      rotation_matrix[0,2] - rotation_matrix[2,0],
                      rotation_matrix[1,0] - rotation_matrix[0,1]])
    axis = axis/np.linalg.norm(axis)
    return axis*angle

def poseToMatrix(pose):
    # Pose is [position, ori]
    T = np.eye(4)
    T[:-1, -1] = np.array(pose[:3])
    T[:-1, :-1] = axisAngleToRotationMatrix(pose[3:])
    return T

def matrixToPose(T):
    pose = np.zeros((6,))
    pose[:3] = T[:-1, -1]
    pose[3:] = rotationMatrixToAxisAngle(T[:-1, :-1])
    return pose

def invertTransformMatrix(T):
    out = np.eye(4)
    out[:-1, :-1] = np.transpose(T[:-1, :-1])
    out[:-1,  -1] = -np.dot(out[:-1, :-1], T[:-1,  -1])
    return out

def segmentColorAndGetKeyPoints(img, hsv_min, hsv_max, draw_contours=False):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask  = cv2.inRange(hsv , hsv_min, hsv_max)
    mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))

    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts = imutils.grab_contours(cnts)

    centroids = []
    for c in cnts:
        # compute the center of the contour
        M = cv2.moments(c)
        if M["m00"] == 0:
            cX = M["m10"]
            cY = M["m01"]
        else:
            cX = M["m10"] / M["m00"]
            cY = M["m01"] / M["m00"]
        centroids.append(np.array([cX, cY]))

    if draw_contours:
        cv2.drawContours(img, cnts, -1, (0, 0, 255), thickness=2)
    
    return np.array(centroids), img