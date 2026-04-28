import numpy as np

def process_noise():
    '''
        Process noise covariance for the EKF. 
        Tune these values based on expected rover dynamics and uncertainties.
    '''
    Q = np.diag([0.01, 0.01, 0.1, 0.1, 0.01])       # [x, y, vx, vy, yaw]
    return Q


def gps_noise():
    '''
        Measurement noise covariance for GPS measurements (used in EKF update).
        Tune these values based on the expected GPS accuracy.
    '''
    R = np.diag([0.5, 0.5])       # [x, y] position noise in meters
    return R

def odom_noise():
    '''
        Measurement noise covariance for odometry measurements (used in EKF update).
        Tune these values based on the expected odometry accuracy.
    '''
    R = np.diag([0.1, 0.1])       # [vx, vy] velocity noise in m/s
    return R