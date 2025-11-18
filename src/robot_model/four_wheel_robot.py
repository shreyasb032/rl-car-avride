import numpy as np

class FourWheelRobot:
    def __init__(self, cfg: dict, time_step: float):
        """
        cfg is expected to be the 'robot' section of your YAML.
        Example keys: length, width, max_steer_angle, max_steer_rate, ...
        time_step: simulation timestep (s)
        """
        # Geometry
        self.L = cfg["length"]
        self.W = cfg["width"]

        # Limits
        self.max_steer_angle = cfg["max_steer_angle"]
        self.max_steer_rate  = cfg["max_steer_rate"]
        self.max_wheel_speed = cfg["max_wheel_speed"]
        self.max_wheel_accel = cfg["max_wheel_accel"]

        # Precompute wheel corner offsets in body frame
        self.wheel_offsets = np.array([
            [+self.L/2, +self.W/2],  # FL
            [+self.L/2, -self.W/2],  # FR
            [-self.L/2, +self.W/2],  # RL
            [-self.L/2, -self.W/2],  # RR
        ])

        # Simulation timestep
        self.dt = time_step

        # Initialize state
        self.reset_state()


    def reset_state(self, x=0.0, y=0.0, theta=0.0):
        # Pose of robot center
        self.x = x
        self.y = y
        self.theta = theta

        # Wheel-level states
        self.steering_angles = np.zeros(4)   # rad
        self.steering_rates  = np.zeros(4)   # rad/s
        self.wheel_speeds    = np.zeros(4)   # m/s
        self.wheel_accels    = np.zeros(4)   # m/s^2

    def set_wheel_commands(self, steer_cmds, speed_cmds):
        """
        Clip desired wheel steering angles/speeds to limits,
        compute steering_rates and wheel_accels for next time step.
        steer_cmds, speed_cmds: arrays of length 4
        """
        # Clip target angles
        target_angles = np.clip(steer_cmds,
                                -self.max_steer_angle,
                                 self.max_steer_angle)

        # Compute required rate to reach targets (rate limit enforcement done later)
        angle_diffs = target_angles - self.steering_angles
        desired_rates = angle_diffs / self.dt
        # Clip rates
        self.steering_rates = np.clip(desired_rates,
                                      -self.max_steer_rate,
                                       self.max_steer_rate)

        # Clip wheel speeds
        target_speeds = np.clip(speed_cmds,
                                -self.max_wheel_speed,
                                 self.max_wheel_speed)
        speed_diffs = target_speeds - self.wheel_speeds
        desired_accels = speed_diffs / self.dt
        self.wheel_accels = np.clip(desired_accels,
                                    -self.max_wheel_accel,
                                     self.max_wheel_accel)

    def get_wheel_global_positions(self):
        """
        Returns 4x2 array of each wheel's (x,y) in world frame.
        """
        R = np.array([[np.cos(self.theta), -np.sin(self.theta)],
                      [np.sin(self.theta),  np.cos(self.theta)]])
        return (R @ self.wheel_offsets.T).T + np.array([self.x, self.y])

    def get_wheel_orientations(self):
        """
        Returns array of 4 angles: global heading of each wheel,
        i.e. body θ plus steering_angles[i].
        """
        return self.theta + self.steering_angles

    # You can add more helpers like:
    #  - get_body_corners()
    #  - full_state_vector()
    #  - set_pose(x,y,theta)
    #  - visualize(self, ax)       # draw on Matplotlib axes
