import numpy as np
from scipy import optimize
from scipy.stats import norm
from .utils import *

# Example initialization function for ParticleFilter class, kwargs would include std
def sampleNormalDistribution(std):
    m = np.zeros(std.shape)
    sample = norm.rvs(m, std)
    prob_individual = norm.pdf(sample, m, std)
    prob_individual[np.isnan(prob_individual)] = 1.0
    return sample, np.prod(prob_individual)

# Example motion model function for Particle Filter class, kwargs would include std
def additiveGaussianNoise(state, std):
    sample, prob = sampleNormalDistribution(std)
    return state + sample, prob

# Create "Lumped Error" Motion model that:
#   1. Includes uncertainty from hand-eye transform
#   2. Distributes uncertainty from joint angles
#   3. sate is [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, e_nb, ..., e_n]
#       where pos, ori is position and axis/angle rep of lumped error
#       e_nb+1, ..., e_n are the errors of the tracked joint angles
#       For more details check: https://arxiv.org/pdf/2102.06235.pdf
#   4. robot_arm is RobotLink that is being tracked
def lumpedErrorMotionModel(state, std_pos, std_ori, robot_arm, std_j, nb):
    # Gaussian uncertainty in hand-eye transform and joint angles
    std = np.concatenate((np.array([std_pos, std_pos, std_pos, std_ori, std_ori, std_ori]), std_j))
    sample, prob = sampleNormalDistribution(std)
    T = poseToMatrix(sample[:6] + state[:6])
    
    # propogate joint error from lumped error which is only up to nb jonts
    T = np.dot(T, robot_arm.propogateJointErrorToLumpedError(sample[6:nb+6]))

    # Return the new state (with added uncertainty on the tracked joint angles)
    return np.concatenate((matrixToPose(T), sample[nb+6:] + state[6:])), prob
    
# State is: [pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, e_nb, ..., e_n]
# where pos, ori is position and axis/angle rep of lumped error
# e_nb+1, ..., e_n are the errors of the tracked joint angles
def pointFeatureObs(state, point_detections, robot_arm, joint_angle_readings, cam, cam_T_b, gamma, association_threshold=20):
    # Get lumped error
    T = poseToMatrix(state[:6])

    # Add estimated joint errors to the robot link
    joint_angle_readings[-(state.shape[0]-6):] += state[6:]
    robot_arm.updateJointAngles(joint_angle_readings)

    # Project points
    p_b,_ = robot_arm.getPointFeatures()
    p_c = np.dot(np.dot(cam_T_b, T), np.transpose(np.concatenate((p_b, np.ones((p_b.shape[0], 1))), axis=1)))
    p_c = np.transpose(p_c)[:, :-1]
    projected_points = cam.projectPoints(p_c)
    
    # Raise error if number of cameras doesn't line up
    if len(projected_points) != len(point_detections):
        raise ValueError("Length of projected_points is {} but length of points_detections is {}.\n".format(len(projected_points), 
                                                                                                            len(point_detections)) \
                        + "Note that these lengths represent the number of cameras being used.")
    
    # Make association between detected and projected & compute probability
    prob = 1
    for c_idx, proj_point in enumerate(projected_points):
        # Use hungarian algorithm to match projected and detected points
        C = np.linalg.norm(proj_point[:, None, :] - point_detections[c_idx][None, :,  :], axis=2)
        row_idx, col_idx = optimize.linear_sum_assignment(C)
        
        # Use threshold to remove outliers
        idx_to_keep = C[row_idx, col_idx] < association_threshold
        row_idx = row_idx[idx_to_keep]
        col_idx = col_idx[idx_to_keep]
        
        # Compute observation probability
        prob *= np.sum(np.exp(-gamma*C[row_idx, col_idx])) \
                + (proj_point.shape[0] - len(row_idx))*np.exp(-gamma*association_threshold)
        
    return prob