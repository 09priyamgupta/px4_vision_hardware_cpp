import numpy as np
import cvxpy as cp

class MPCTracker:
    def __init__(self, horizon=20, dt=0.05, fov_deg=40.0, altitude=3.0):
        self.N = horizon
        self.dt = dt
        self.nx = 2             # State: [rel_x, rel_y]
        self.nu = 2             # Input: [vx_cmd, vy_cmd]

        # --- Constraints ---
        # 1. FOV Constraint (80% of actual FOV for a safety margin)
        self.fov_rad = np.deg2rad(fov_deg / 2.0)

        # Dynamic FOV Constraint: Max position error allowed based on altitude and camera FOV
        self.fov_bound = cp.Parameter(nonneg=True)  
        
        # 2. Max Drone Speed (m/s)
        self.max_vel = 1.0  

        print(f"[MPC] Initialized. Max Vel < {self.max_vel}m/s. FOV adapts dynamically.")

        # --- Optimization Variables ---
        self.X = cp.Variable((self.nx, self.N + 1))     # Predicted Positions
        self.U = cp.Variable((self.nu, self.N))         # Computed Velocities
        self.S = cp.Variable((self.nx, self.N + 1))     # Slack variable for soft constraints

        # --- Parameters (Inputs to solver) ---
        self.init_state = cp.Parameter(self.nx)     # Current Relative Pos
        self.tag_vel = cp.Parameter(self.nu)        # Current Tag Velocity Estimate
        self.prev_u = cp.Parameter(self.nu)         # Previous control input (velocity coomand) for smoothness

        # --- Cost Weights ---
        Q = np.eye(self.nx) * 8.0          # Position Error: Keep it moderate so it doesn't rush
        R = np.eye(self.nu) * 14.0           # Velocity Effort: Penalize deviating from tag speed
        R_diff = np.eye(self.nu) * 10.0      # Jerk Penalty: MASSIVE weight to ensure smooth braking
        W_slack = 1000.0                    
        # Slack Penalty: Massive weight to enforce FOV bounds

        # --- Build the Problem ---
        constraints = [
                        self.X[:, 0] == self.init_state,
                        self.S >= 0     # Slack must be positive
                    ]
        cost = 0

        for k in range(self.N):
            # Dynamics: Tag velocities AND disturbance estmates 
            # x[k+1] = x[k] + (u[k] - v_tag) * dt
            constraints += [
                self.X[:, k+1] == self.X[:, k] + (self.U[:, k] - self.tag_vel) * self.dt
            ]

            # Soft FOV Constraint: |Position| <= Maximum FOV + Slack
            constraints += [self.X[:, k+1] <= self.fov_bound + self.S[:, k+1]]
            constraints += [self.X[:, k+1] >= -self.fov_bound - self.S[:, k+1]]

            # Hard Velocity Constraint: Physical motor limits
            constraints += [self.U[:, k] <= self.max_vel]
            constraints += [self.U[:, k] >= -self.max_vel]

            # -------------------- Cost Function --------------------
            # Minimize position error
            cost += cp.quad_form(self.X[:, k+1], Q)

            # Minimize velocity difference (Drone Vel should ≈ Tag Vel)
            cost += cp.quad_form(self.U[:, k] - self.tag_vel, R)

            # Penalize violating FOV constraint (Slack variable)
            cost += W_slack * cp.sum(self.S[:, k+1])

            # Jerk Penalty (Smoothness): Penalize changes in velocity commands
            if k == 0:
                cost += cp.quad_form(self.U[:, k] - self.prev_u, R_diff)
            else:
                cost += cp.quad_form(self.U[:, k] - self.U[:, k-1], R_diff)
            
        # --------------- Problem Definition ---------------
        self.problem = cp.Problem(cp.Minimize(cost), constraints)

        # State memory for smooth continuous control
        self.last_u_val = np.zeros(self.nu)
        self.last_d_est = np.zeros(self.nx)


    def get_predictions(self, N=5):
        '''
            Returns the first N steps of predicted positions and velocities
        '''
        if self.X.value is not None and self.U.value is not None:
            # X shape is (2, N+1), U shape is (2, N)
            return self.X.value[:, :N], self.U.value[:, :N]
        return None, None

    def solve(self, current_pos, current_tag_vel, current_altitude):
        '''
            current_pos: [px, py] relative to tag (rotated to gravity-aligned frame)
            current_tag_vel: [vx, vy] absolute velocity of the tag (from Kalman Filter)
            current_altitude: Positive float representing height above target
        '''
        self.init_state.value = current_pos
        self.tag_vel.value = current_tag_vel
        self.prev_u.value = self.last_u_val

        # Dynamically calculate the FOV footprint based on actual altitude and camera FOV
        # We clamp at 0.2m minimum to prevent the bounding box from becoming too small at very low altitudes 
        safe_altitude = max(current_altitude, 0.5)

        # We will use 90% of the theoretical FOV
        strict_fov = safe_altitude * np.tan(self.fov_rad) * 0.9

        # Dynamic Safety Margin
        dynamic_margin = max(0.5, 1.5 - (current_altitude * 0.25))

        self.fov_bound.value = strict_fov + dynamic_margin  # Add dynamic safety margin

        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
        
        except Exception as e:
            print(f"[MPC] Solver Crash: {e}")
            return self.last_u_val  # Return safe previous command

        if self.problem.status not in ["optimal", "optimal_inaccurate"]:
            print(f"[MPC] Solver failed: {self.problem.status}")
            return self.last_u_val  # Return safe previous command
        
        # Update last_u_val for next iteration
        self.last_u_val = self.U[:, 0].value

        # Return optimal velocity vector
        return self.U[:, 0].value