import numpy as np

def rover_motion_model(x, u, dt):
    '''
        Simple kinematic motion model for the rover. 

        State vector x: [x, y, vx, vy, yaw]
        Control input u: [yaw_rate]
    '''
    yaw = x[4]
    yaw_rate = u[0]

    x_new = x.copy()
    x_new[0] += x[2] * dt
    x_new[1] += x[3] * dt
    x_new[4] += yaw_rate * dt

    return x_new

def rover_jacobian_F(x, dt):
    '''
        Jacobian of the rover motion model w.r.t state x
    '''
    F = np.eye(5)
    F[0, 2] = dt
    F[1, 3] = dt
    return F