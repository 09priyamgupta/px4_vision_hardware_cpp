import numpy as np
import math

def rmse(error):
    '''
        Compute the Root Mean Square Error (RMSE) given an array of errors.
    '''
    error = np.array(error)
    return np.sqrt(np.mean(error**2))

def mean_absolute_error(error):
    '''
        Compute the Mean Absolute Error (MAE) given an array of errors.
    '''
    error = np.array(error)
    return np.mean(np.abs(error))

def landing_error(drone_xy, tag_xy):
    '''
        Compute the landing error as the distance between the drone final position and the tag position.
    '''
    dx = drone_xy[0] - tag_xy[0]
    dy = drone_xy[1] - tag_xy[1]

    error = math.sqrt(dx**2 + dy**2)
    return error

def tracking_rmse(rel_xyz):
    '''
        Compute the RMSE of the relative position estimates.
        Measure how noisy the relative pose estimates are using RMSE.
    '''
    rmse_x = rmse(rel_xyz[:, 0])
    rmse_y = rmse(rel_xyz[:, 1])
    rmse_z = rmse(rel_xyz[:, 2])
    
    return (rmse_x, rmse_y, rmse_z)

def tag_position_stability(tag_xyz):
    '''
        Compute the standard deviation of the AprilTag position estimates in the map frame.
        Lower standard deviation indicates better stability.
    '''
    std_xyz = np.std(tag_xyz, axis=0)
    return std_xyz