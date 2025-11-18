import numpy as np
import pytest
from robot_model.four_wheel_robot import FourWheelRobot
from kinematics.kinematics import curvature_to_wheel_commands, FourWheelKinematics

@pytest.fixture
def cfg_robot():
    # minimal dict matching your YAML robot section
    return {
        "length": 1.0,
        "width":  0.6,
        "max_steer_angle": np.pi/4,
        "max_steer_rate":  np.pi,
        "max_wheel_speed": 1.5,
        "max_wheel_accel": 3.0
    }

def test_straight_line(cfg_robot):
    r = FourWheelRobot(cfg_robot, time_step=0.1)
    offsets = r.wheel_offsets
    phi, vels = curvature_to_wheel_commands(offsets, curvature=0.0, velocity=1.23)
    assert np.allclose(phi, 0.0)
    assert np.allclose(vels, 1.23)

def test_circle_turn(cfg_robot):
    r = FourWheelRobot(cfg_robot, time_step=0.1)
    # pick k so R=5m, v=1.0m/s
    k = 1/5
    phi, vels = curvature_to_wheel_commands(r.wheel_offsets, k, 1.0)
    # verify wheel speeds ratio matches radius ratio
    r0 = np.hypot(*r.wheel_offsets[0] - np.array([0,5]))
    r1 = np.hypot(*r.wheel_offsets[1] - np.array([0,5]))
    assert np.isclose(vels[0]/vels[1], r0/r1)
    # steering angles should differ front vs rear
    assert phi[0] != phi[2]
