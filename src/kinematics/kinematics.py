import numpy as np
from robot_model.four_wheel_robot import FourWheelRobot

def curvature_to_wheel_commands(offsets, curvature, velocity):
    """
    offsets: (4,2) array of (x_i, y_i) wheel positions in body frame
    curvature: κ (1/m)
    velocity: forward speed at center (m/s)
    returns: steer_cmds (4,), speed_cmds (4,)
    """
    N = offsets.shape[0]
    steer_cmds = np.zeros(N)
    speed_cmds = np.zeros(N)

    # Straight‐line fallback
    if abs(curvature) < 1e-6:
        steer_cmds[:] = 0.0
        speed_cmds[:] = velocity
        return steer_cmds, speed_cmds

    R = 1.0 / curvature       # signed turning radius
    omega = velocity * curvature  # yaw rate

    for i, (x_i, y_i) in enumerate(offsets):
        # compute steering angle for wheel i
        steer_cmds[i] = np.arctan2(x_i, R - y_i)
        # compute individual wheel radius r_i
        r_i = np.hypot(x_i, R - y_i)
        # linear speed v_i
        speed_cmds[i] = np.sign(velocity) * abs(omega) * r_i

    return steer_cmds, speed_cmds


class FourWheelKinematics:
    def __init__(self, robot: FourWheelRobot):
        self.robot = robot

    def step(self, curvature, velocity):
        """
        1) Compute wheel-level commands
        2) Enforce limits in robot.set_wheel_commands()
        3) Integrate wheel states
        4) Update robot center pose
        """
        # 1) map (k,v) to per‐wheel targets
        steer_cmds, speed_cmds = curvature_to_wheel_commands(
            self.robot.wheel_offsets,
            curvature, velocity
        )

        # 2) clip to robot limits & compute rates/accels
        self.robot.set_wheel_commands(steer_cmds, speed_cmds)

        # 3) integrate wheel states
        dt = self.robot.dt
        self.robot.steering_angles += self.robot.steering_rates * dt
        self.robot.wheel_speeds    += self.robot.wheel_accels * dt

        # 4) integrate the robot center pose (simple bicycle‐like update)
        dx = velocity * np.cos(self.robot.theta) * dt
        dy = velocity * np.sin(self.robot.theta) * dt
        dtheta = velocity * curvature * dt

        self.robot.x     += dx
        self.robot.y     += dy
        self.robot.theta += dtheta
