
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from enum import Enum

import modern_robotics as mr
import numpy as np


def example_code():

    # The robot object is what you use to control the robot
    robot = InterbotixManipulatorXS("px100", "arm", "gripper")
    robot_startup()
    mode = 'h'
    # Let the user select the position
    while mode != 'q':
        mode = input("[h]ome, [s]leep, [q]uit ")
        if mode == "h":
            robot.arm.go_to_home_pose()
        elif mode == "s":
            robot.arm.go_to_sleep_pose()

    robot_shutdown()


class DIRECTION(Enum):
    UP = 0,
    DOWN = 1,
    FORWARD = 2,
    BACKWARD = 3
    LEFT = 4,
    RIGHT = 5


class Controller:

    def __init__(self) -> None:
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")
        robot_startup()
        self.joint_dict = {}
        self.update_joint_angle_dict()
        self.step_size = 10  # cm

    def update_joint_angle_dict(self):
        angles = self.robot.arm.get_joint_commands()
        self.joint_dict["waist"] = angles[0]
        self.joint_dict["shoulder"] = angles[1]
        self.joint_dict["elbow"] = angles[2]
        self.joint_dict["wrist_angle"] = angles[3]

    def ee_pos(self):
        return self.robot.arm.get_ee_pose_command()[3]

    def parse_inputs(self):
        cmd_str = f"\nEnter a command for the robot using one of the commands, some will prompt for more arguments" + \
            f"\nWaist: {np.rad2deg(self.joint_dict["waist"]):.2f}\N{DEGREE SIGN}" + \
            f"\nShoulder: {np.rad2deg(self.joint_dict["shoulder"]):.2f}\N{DEGREE SIGN}" + \
            f"\nElbow: {np.rad2deg(self.joint_dict["elbow"]): .2f}\N{DEGREE SIGN}" + \
            f"\nWrist Angle: {np.rad2deg(self.joint_dict["wrist_angle"]):.2f}\N{DEGREE SIGN}" + \
            f"\nEE Position: ({self.ee_pos()[0]:.2f}, {self.ee_pos()[1]:.2f}, {self.ee_pos()[3]:.2f})" + \
            f"\n[h]ome, [s]leep, [q]uit, [u]p, [d]own, [f]orward, [b]ackward, [st]ep_size: "
        cmd = input(cmd_str).lower()
        if cmd == 'q' or cmd == 'quit':
            return False
        elif cmd == 'h' or cmd == 'home':
            print(f"Moving to home position...")
            self.robot.arm.go_to_home_pose()
        elif cmd == 's' or cmd == 'sleep':
            print(f"Moving to sleep position...")
            self.robot.arm.go_to_sleep_pose()
        elif cmd == 'u' or cmd == 'up':
            self.move(DIRECTION.UP, self.step_size)
        elif cmd == 'd' or cmd == 'down':
            self.move(DIRECTION.UP, self.step_size)
        elif cmd == 'f' or cmd == 'forward':
            self.move(DIRECTION.UP, self.step_size)
        elif cmd == 'b' or cmd == 'backward':
            self.move(DIRECTION.UP, self.step_size)
        elif cmd == 'st' or cmd == 'step_size':
            new_step_size = input(f"Please enter a new step size [cm]: ")
            self.step_size = new_step_size
        self.update_joint_angle_dict()
        return True

    def open_gripper(self):
        self.robot.gripper.release()

    def close_gripper(self):
        self.robot.gripper.grasp()

    def joint_positions(self):
        return self.robot.arm.get_joint_commands()

    def move(self, dir: DIRECTION, relative_distance: float):
        match dir:
            case DIRECTION.UP:
                pass
            case DIRECTION.DOWN:
                pass
            case DIRECTION.FORWARD:
                pass
            case DIRECTION.BACKWARD:
                pass
            case DIRECTION.LEFT:  # Rotate left
                pass
            case DIRECTION.RIGHT:  # Rotate right
                pass
            case _:
                pass

    def move_toward(self, point):
        # Get current ee position
        # Get X, Y, Z displacement betweeen current and desired point and set trajectory
        current_pos = self.ee_pos()
        self.robot.arm.set_ee_cartesian_trajectory(point[0] - current_pos[0],
                                                   point[1] - current_pos[1],
                                                   point[2] - current_pos[2],)

    def forward_kinematics(self, joint_positions):
        T = mr.FKinSpace(self.robot.arm.robot_des.M,
                         self.robot.arm.robot_des.Slist, joint_positions)
        return mr.TransToRp(T)  # [R, p]

    def shutdown(self):
        robot_shutdown()


if __name__ == '__main__':
    controller = Controller()
    continueControl = True
    while continueControl:
        continueControl = controller.parse_inputs()
    controller.shutdown()
