import numpy as np

class EKF:
    def __init__(self, x0, P0, Q, R):
        self.x = x0      # State estimate
        self.P = P0      # Estimate covariance
        self.Q = Q       # Process noise covariance
        self.R = R       # Measurement noise covariance

    def predict(self, f, F, u, dt):
        '''
            f: state transition function
            F: Jacobian of f w.r.t state
            u: control input
            dt: time step
        '''
        # Predict state
        self.x = f(self.x, u, dt)

        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z, h, H, R):
        '''
            z: measurement
            h: measurement function
            H: Jacobian of h w.r.t state
            R: measurement noise covariance
        '''
        # Measurement prediction
        z_pred = h(self.x)

        # Measurement residual
        y = z - z_pred

        # Residual covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state estimate
        self.x = self.x + K @ y

        # Update estimate covariance
        I = np.eye(len(self.x))
        self.P = (I - K @ H) @ self.P