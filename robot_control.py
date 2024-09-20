
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
    BACKWARD = 3,
    ROTATE = 4


class Controller:

    def __init__(self) -> None:
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")
        robot_startup()
        self.joint_dict = {}
        self.update_joint_angle_dict()
        self.step_size = 5 / 100  # default is 5 cm, convert to meters

    def update_joint_angle_dict(self):
        angles = self.robot.arm.get_joint_commands()
        self.joint_dict["waist"] = angles[0]
        self.joint_dict["shoulder"] = angles[1]
        self.joint_dict["elbow"] = angles[2]
        self.joint_dict["wrist_angle"] = angles[3]

    def ee_pos(self):
        T = self.robot.arm.get_ee_pose()
        return (T[0][3], T[1][3], T[2][3])

    def parse_inputs(self):
        ee_pos = self.ee_pos()
        cmd_str = f"\nEnter a command for the robot using one of the commands, some will prompt for more arguments" + \
            f"\nWaist: {np.rad2deg(self.joint_dict["waist"]):.2f}\N{DEGREE SIGN}" + \
            f"\nShoulder: {np.rad2deg(self.joint_dict["shoulder"]):.2f}\N{DEGREE SIGN}" + \
            f"\nElbow: {np.rad2deg(self.joint_dict["elbow"]): .2f}\N{DEGREE SIGN}" + \
            f"\nWrist Angle: {np.rad2deg(self.joint_dict["wrist_angle"]):.2f}\N{DEGREE SIGN}" + \
            f"\nEE Position: ({100 * ee_pos[0]:.2f}, {100 * ee_pos[1]:.2f}, {100 * ee_pos[2]:.2f}) cm" + \
            f"\n[h]ome, [s]leep, [q]uit, [u]p, [d]own, [f]orward, [b]ackward, [st]ep size (meters), [r]otate (deg), [p]oint: "
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
            self.move(DIRECTION.DOWN, self.step_size)
        elif cmd == 'f' or cmd == 'forward':
            self.move(DIRECTION.FORWARD, self.step_size)
        elif cmd == 'b' or cmd == 'backward':
            self.move(DIRECTION.BACKWARD, self.step_size)
        elif cmd == 'r' or cmd == 'rotate':
            angle_deg = input(
                f"Please enter the amount of degrees you want to rotate (absolute position): ")
            angle = float(angle_deg)
            self.move(DIRECTION.ROTATE, angle)
        elif cmd == 'st' or cmd == 'step_size':
            new_step_size = input(f"Please enter a new step size [m]: ")
            self.step_size = float(new_step_size)
        elif cmd == 'p' or cmd == 'point':
            point = input(
                f"Please input an absolute point [cm] as a tuple of (x,y,z): ")
            points = point.replace("(", "").replace(")", "").split(",")
            points = [float(p) / 100 for p in points]
            self.move_to_point(points)
        else:
            print(f"Invalid cmd given: {cmd}")
        self.update_joint_angle_dict()
        return True

    def open_gripper(self):
        self.robot.gripper.release()

    def close_gripper(self):
        self.robot.gripper.grasp()

    def joint_positions(self):
        return self.robot.arm.get_joint_commands()

    def move(self, dir: DIRECTION, relative_distance: float):
        current_ee_pos = self.ee_pos()
        match dir:
            case DIRECTION.UP:
                self.robot.arm.set_ee_pose_components(
                    current_ee_pos[0], current_ee_pos[1], current_ee_pos[2] + relative_distance)
            case DIRECTION.DOWN:
                self.robot.arm.set_ee_pose_components(
                    current_ee_pos[0], current_ee_pos[1], current_ee_pos[2] - relative_distance)
            case DIRECTION.FORWARD:
                self.robot.arm.set_ee_pose_components(
                    current_ee_pos[0] + relative_distance, current_ee_pos[1], current_ee_pos[2])
            case DIRECTION.BACKWARD:
                self.robot.arm.set_ee_pose_components(
                    current_ee_pos[0] - relative_distance,  current_ee_pos[1], current_ee_pos[2])
            case DIRECTION.ROTATE:
                return self.robot.arm.set_single_joint_position(
                    "waist", np.deg2rad(relative_distance))
            case _:
                print(f"Invalid movement direction given")
                return False

    def move_to_point(self, point):
        self.robot.arm.set_ee_pose_components(
            x=point[0], y=point[1], z=point[2])

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
