from utils.config import Config
from robot_model.four_wheel_robot import FourWheelRobot

def main(config_path="configs/default.yaml"):
    # 1) Load all configs
    cfg = Config(config_path)

    # 2) Extract just the robot section
    robot_cfg = cfg["robot"]

    # 3) Instantiate robot  
    robot = FourWheelRobot(robot_cfg, time_step=cfg["env"]["time_step"])

    # Now robot is fully parameterized from YAML!
    print(f"Robot length = {robot.L}, max steer rate = {robot.max_steer_rate}")

if __name__ == "__main__":
    main()
