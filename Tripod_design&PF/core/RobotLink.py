import numpy as np
import json
from .utils import *

class RobotLink:
    # To reduce number of repeated computations, this class updates internal data structures
    # with the updateJointAngles function and then uses "get" functions to access the data
    def __init__(self, json_file_path):
        # Load robot DH parameter information
        # This only supports modified DH parameters
        f = open(json_file_path)
        robot_json_library = json.load(f)
        
        # Internal data structures to hold all the robot information
        self.alpha = []
        self.A     = []
        self.theta = []
        self.D     = []
        self.type  = []
        self.Tx    = []
        self.joint_angles = []
        self.jointT = []
        self.baseToJointT = []
        
        for idx, dh in enumerate(robot_json_library["DH_params"]):
            self.type.append(dh["type"])
            self.A.append(dh["A"])
            self.alpha.append(dh["alpha"])

            if dh["type"] == "revolute": 
                self.theta.append(dh["theta"] + dh["offset"])
                self.D.append(dh["D"])
            elif dh["type"] == "prismatic": 
                self.theta.append(dh["theta"])
                self.D.append(dh["D"] + dh["offset"])
            else: 
                raise ValueError("Type {} is not defined.".format(dh["type"]))

            # Pre-generate the Tx matrices
            ca = np.cos(dh["alpha"])
            sa = np.sin(dh["alpha"])
            self.Tx.append(np.array([[1.0, 0.0, 0.0, dh["A"]],
                                     [0.0,  ca, -sa, 0.0],
                                     [0.0,  sa,  ca, 0.0],
                                     [0.0, 0.0, 0.0, 1.0]]))
            
            # Initialize joint transforms
            self.joint_angles.append(-1.0)
            self.jointT.append(np.eye(4))
            self.baseToJointT.append(np.eye(4))
            
        # Right now all the gripper stuff is hard-coded for psm surgical tools
        # Since this whole class just provides positions relative to joint angles,
        # just change the "calculateGripperTransform" function if a different gripper is needed
        self.has_gripper = robot_json_library["has_gripper"]
        if self.has_gripper:
            self.joint_angles.append(-1.0)
            gripperT = self.calculateGripperTransform(0.0)
            self.jointT = self.jointT + gripperT
            self.baseToJointT = self.baseToJointT + [np.eye(4) for _ in range(len(gripperT))]
               
        # Load point and cylinder features
        self.point_features   = robot_json_library["point_features"]
        self.point_features3D = None
        self.point_featuresNames = []
        for point in self.point_features:
            self.point_featuresNames.append(point["name"])
        
        self.shaft_features   = robot_json_library["shaft_features"]
        self.shaft_features3DPos = None
        self.shaft_features3DDir = None
        self.shaft_featuresNames = []
        self.shaft_radius = []
        for shaft in self.shaft_features:
            self.shaft_featuresNames.append(shaft["name"])
            self.shaft_radius.append(shaft["radius"])
            
        # Load skeleton features here (this is helpful for quick visualization of the robot)
        self.skeleton_structure = robot_json_library["skeleton_structure"]
        self.skeleton_3DPoints  = None
        
        # Initialize all the data structures at 0 joint angles
        self.updateJointAngles([0.0 for _ in range(len(self.joint_angles))])
    
    def getPointFeatures(self):   
        # returns self.point_features3D
        # data type is np array Nx3 for N point features on the robot arm
        return self.point_features3D, self.point_featuresNames
    
    def getShaftFeatures(self):
        # returns self.shaft_features3DPos and self.shaft_features3DDir
        # data type for both is Nx3 numpy array for N shaft features 
        return self.shaft_features3DPos, self.shaft_features3DDir, self.shaft_featuresNames, self.shaft_radius
    
    def getSkeletonPoints(self):
        # data type is [(p11, p12), (p21, p22), ... , (pN1, pN2)]
        # each tupple (pK1, pK2) defines a line on the skeleton 
        # and each point in the tuple is a 3D point
        return self.skeleton_3DPoints
        
    def updateJointAngles(self, joint_angles):
        if len(joint_angles) != len(self.joint_angles):
            raise ValueError("Incorrect number of joint angles, should be {} is {}.".format(len(self.joint_angles), 
                                                                                            len(joint_angles)))
        no_change_in_joint_angles = True
        
        for idx, angle in enumerate(joint_angles):
            if angle == self.joint_angles[idx] and no_change_in_joint_angles:
                continue
            no_change_in_joint_angles = False
            self.joint_angles[idx] = angle
            
            # Case gripper calculation
            if idx == len(joint_angles) - 1 and self.has_gripper:
                gripperT = self.calculateGripperTransform(self.joint_angles[idx])
                
                for gripper_idx, T in enumerate(gripperT):
                    self.jointT[gripper_idx + idx] = T
                    self.baseToJointT[gripper_idx + idx] = np.dot(self.baseToJointT[idx-1], T)
                
            # Normal DH kinematic chain case  
            else:
                self.jointT[idx] = self.calculateJointTransform(idx, self.joint_angles[idx])
                if idx == 0:
                    self.baseToJointT[idx] = self.jointT[idx]
                else:
                    self.baseToJointT[idx] = np.dot(self.baseToJointT[idx-1],self.jointT[idx])
        
        self.point_features3D  = self.calculateFeaturePoints()
        self.shaft_features3DPos, self.shaft_features3DDir = self.calculateFeatureShafts()
        self.skeleton_3DPoints = self.calculateSkeletonPoints() 
    
    def calculateFeaturePoints(self):
        point_features3D = np.zeros((len(self.point_features), 3))
        for idx, point in enumerate(self.point_features):
            pos = np.array([point["position"][0], point["position"][1], point["position"][2], 1])
            point_features3D[idx] = np.dot(self.baseToJointT[int(point["link"])-1], pos)[:-1]
        return point_features3D
    
    def calculateFeatureShafts(self):
        shaft_features3DPos = np.zeros((len(self.shaft_features), 3))
        shaft_features3DDir = np.zeros((len(self.shaft_features), 3))
        for idx, shaft in enumerate(self.shaft_features):
            pos = np.array([shaft["position"][0], shaft["position"][1], shaft["position"][2], 1])
            shaft_features3DPos[idx] = np.dot(self.baseToJointT[int(shaft["link"])-1], pos)[:-1]

            direction = np.array([shaft["direction"][0], shaft["direction"][1], shaft["direction"][2], 0])
            shaft_features3DDir[idx] = np.dot(self.baseToJointT[int(shaft["link"])-1], direction)[:-1]
        return shaft_features3DPos, shaft_features3DDir

    def calculateSkeletonPoints(self):
        skeleton_3DPoints = []

        for skeleton in self.skeleton_structure:
            pos1 = np.array([skeleton["position1"][0], skeleton["position1"][1], skeleton["position1"][2], 1])
            pos1 = np.dot(self.baseToJointT[int(skeleton["link1"])-1], pos1)[:-1]
            pos2 = np.array([skeleton["position2"][0], skeleton["position2"][1], skeleton["position2"][2], 1])
            pos2 = np.dot(self.baseToJointT[int(skeleton["link2"])-1], pos2)[:-1]
            
            skeleton_3DPoints.append((pos1, pos2))
        
        return skeleton_3DPoints
    
    def calculateJointTransform(self, index, angle):
        if self.type[index] == "revolute":
            theta = self.theta[index] + angle
            D = self.D[index]
        elif self.type[index] == "prismatic":
            theta = self.theta[index] 
            D = self.D[index] + angle
        else:
            raise ValueError("Invalid joint type {} at index {}, should be revolute or prismatic.".format(self.type[index], index))
            
        ct = np.cos(theta)
        st = np.sin(theta)
        T_z = np.array([[ ct, -st, 0.0, 0.0],
                        [ st,  ct, 0.0, 0.0],
                        [0.0, 0.0, 1.0,   D],
                        [0.0, 0.0, 0.0, 1.0]])
        
        return np.dot(self.Tx[index], T_z)
    
    def calculateGripperTransform(self, jaw_angle):
        # Assumes gripper angle is the last element of self.joint_angles
        # Hardcoded for PSM robot arms but can be modified for others
        # so long as the output is a list of transforms to the different parts of the grippers
        # Note the output idx is offset by the number of DH parameters to define link locations for features
        # For example, the LND PSM tool has 6 DH parameters, so link 7 corresponds to the first transform outputted here\
        # also note, the output transforms here are assumed to be in parallel and just serially connected to last
        # transform in the DH parameter chain
        eHeL = np.array([[np.cos(jaw_angle/2.0), -np.sin(jaw_angle/2.0), 0, 0],
                         [np.sin(jaw_angle/2.0),  np.cos(jaw_angle/2.0), 0, 0],
                         [ 0, 0, 1, 0],
                         [ 0, 0, 0, 1]])
        eHeR = np.array([[ np.cos(jaw_angle/2.0),  np.sin(jaw_angle/2.0), 0, 0],
                         [-np.sin(jaw_angle/2.0),  np.cos(jaw_angle/2.0), 0, 0],
                         [ 0, 0, 1, 0],
                         [ 0, 0, 0, 1]])
        
        return [eHeL, eHeR]
    
    def propogateJointErrorToLumpedError(self, j_errors):
        T = np.eye(4)
        for idx, e in enumerate(j_errors):
            
            # Compute as in equation 11 here:
            # https://arxiv.org/pdf/2102.06235.pdf
            if self.type[idx] == "revolute":
                ct = np.cos(e)
                st = np.sin(e)
                Tz = np.array([[ ct, -st, 0.0, 0.0],
                                [ st,  ct, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])
            elif self.type[idx] == "prismatic":
                Tz = np.array([[1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0,   e],
                                [0.0, 0.0, 0.0, 1.0]])
            else:
                raise ValueError("Invalid joint type {} at index {}, should be revolute or prismatic.".format(self.type[idx], idx))
            
            T_e = np.dot( np.dot(self.Tx[idx], Tz), invertTransformMatrix(self.Tx[idx]) )
            if idx > 1:
                T = np.dot( np.dot( np.dot(T, self.baseToJointT[idx-1]), T_e), invertTransformMatrix(self.baseToJointT[idx-1]) )
            else:
                T = np.dot(T, T_e)
        return T