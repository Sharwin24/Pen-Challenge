
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from enum import Enum

import modern_robotics as mr
import numpy as np
import pickle
import os

import scipy.spatial.transform as tr


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


class Calibrator:
    def __init__(self, file_name=None) -> None:
        self.points = []
        self.params = []
        if file_name != None:
            self.load(file_name)

    def add_point(self, p):
        self.points.append(p)

    def remove_point(self, p):
        self.points.remove(p)

    def get_points(self):
        return self.points

    ############# Begin_Citation [9] #############
    def save(self, file_name, points=None, params=None):
        # Write to a file
        if os.path.exists(file_name):
            # If the file exists, clear the data
            print(f"{file_name} already exists, overwriting!")
            os.remove(file_name)
        if points == None:
            points = self.points
        if params == None:
            params = self.params
        with open(file_name, 'wb') as f:
            pickle.dump([self.points, self.params], f)
        print(f"Saved {self.points}\n{self.params}")

    def load(self, file_name):
        # If file exists, load it
        if os.path.exists(file_name):
            with open(file_name, 'rb') as f:
                self.points, self.params = pickle.load(f)
        else:
            print(f"{file_name} does not exist")
    ############# End_Citation [9] #############

    def compute_calibration_parameters(self, robot_points: list, camera_points: list):
        # The passed in points are the list of points (x,y,z) m in the robot and camera frame, respectively
        # Compute the centroid of the points
        # Get the subtracted centroid where each point in the list subtracts the centroid
        # Then figure out the rotation between the subtracted centroids of each frame
        # Find the translation using the rotation matrix and known points (centroids)
        # Store all params in a tuple (R, t)
        robot_centroid = (
            sum([p[0] for p in robot_points]) / len(robot_points),
            sum([p[1] for p in robot_points]) / len(robot_points),
            sum([p[2] for p in robot_points]) / len(robot_points)
        )
        camera_centroid = (
            sum([p[0] for p in camera_points]) / len(camera_points),
            sum([p[1] for p in camera_points]) / len(camera_points),
            sum([p[2] for p in camera_points]) / len(camera_points)
        )
        print(f"R_c {robot_centroid}\nC_c {camera_centroid}")
        subtracted_robot_centroids = [
            (robot_centroid[0] - p[0], robot_centroid[1] - p[1], robot_centroid[2] - p[2]) for p in robot_points]
        subtracted_camera_centroids = [
            (camera_centroid[0] - p[0], camera_centroid[1] - p[1], camera_centroid[2] - p[2]) for p in camera_points]
        print(f"R_sc {subtracted_robot_centroids}\nC_sc" +
              f"{subtracted_camera_centroids}")
        R = tr.Rotation.align_vectors(
            subtracted_robot_centroids, subtracted_camera_centroids)
        rotated_camera = np.array(R[0].as_matrix()) @ np.array(camera_centroid)
        print(f"R @ C_c = {rotated_camera}")
        t = np.subtract(np.array(robot_centroid), rotated_camera)
        print(f"Translation = {t}")
        # t = np.subtract(np.array(robot_centroid),
        #                 np.matmul(np.array(camera_centroid), R))
        # t = (robot_centroid[0] - (R * camera_centroid[0]),
        #      robot_centroid[1] - (R * camera_centroid[1]),
        #      robot_centroid[2] - (R * camera_centroid[2]))
        self.params = [R, t]


class Controller:

    def __init__(self) -> None:
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")
        robot_startup()
        self.robot.gripper.set_pressure(0.25)
        self.joint_dict = {}
        self.update_joint_angle_dict()
        self.step_size = 5 / 100  # default is 5 cm, convert to meters
        self.cb = Calibrator()
        self.visited_points = []

    def home(self):
        self.robot.arm.go_to_home_pose()

    def sleep(self):
        self.robot.arm.go_to_sleep_pose()

    def start(self):
        continueControl = True
        while continueControl:
            continueControl = self.parse_inputs()
        self.shutdown()

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
            f"\n[h]ome, [s]leep, [q]uit, [u]p, [d]own, [f]orward, [b]ackward, [st]ep size (meters), [r]otate (deg), [p]oint, [c]alibrate, [v]isiting, [re]lease, [g]rasp: "
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
            point_tuple = point.replace("(", "").replace(")", "").split(",")
            point_tuple = [float(p) / 100 for p in point_tuple]
            self.move_to_point(point_tuple)
        elif cmd == 'c' or cmd == 'calibrate':
            self.calibrate()
        elif cmd == 'v' or cmd == 'visiting':
            self.visiting()
        elif cmd == 're' or cmd == 'release':
            self.open_gripper()
        elif cmd == 'g' or cmd == 'grasp':
            self.close_gripper()
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
            x=point[0], y=point[1], z=point[2], moving_time=1)

    def forward_kinematics(self, joint_positions):
        T = mr.FKinSpace(self.robot.arm.robot_des.M,
                         self.robot.arm.robot_des.Slist, joint_positions)
        return mr.TransToRp(T)  # [R, p]

    def visiting(self):
        # Visit points and if saving is desired, allow the user to save it
        visiting = True
        while visiting:
            point = input(
                f"Please input an absolute point [cm] as a tuple of (x,y,z): ")
            point_tuple = point.replace(
                "(", "").replace(")", "").split(",")
            point_tuple = [float(p) / 100 for p in point_tuple]
            self.move_to_point(point_tuple)
            save = input(
                f"Save this point to calibration_sequence? {point_tuple} [y,n]: ").lower()
            if save == 'y':
                print(f"Saving {point_tuple}")
                self.visited_points.append(point_tuple)
            visiting = input("Continue? [y,n]: ").lower() == 'y'
            if not visiting:
                print(f"Calibration Path: {self.visited_points}")

    def calibrate(self):
        # Calibration Procedure
        # The script should be run while the robot is holding a pen
        # The robot should move to a few pre-set positions to gather the necessary data
        # The script then computes and outputs the calibration parameters
        # The robot moves to a few test locations and the camera coordinates are converted to the robot coordinates and compared
        print(f"Starting Calibration Procedure")
        holding = input(
            f"Is the robot holding the pen already, if not gripper will open? [y,n]: ")
        if holding == 'n':
            self.robot.gripper.release()
            _ = input(f"Press enter to grab the pen")
            self.robot.gripper.grasp()
        calib_file = input(
            "Enter the file name for the calibration points to load from\nIf it doesn't exist it will be ignored: ")
        self.cb.load(calib_file)
        points_to_visit = self.cb.get_points()
        if len(points_to_visit) == 0 and len(self.visited_points) != 0:
            # If no points were loaded, and visiting loop was run, use those points
            print(f"Loading points from visiting loop:" +
                  f"\n{self.visited_points}")
            points_to_visit.extend(self.visited_points)
        elif len(points_to_visit) == 0 and len(self.visited_points) == 0:
            # If no points at all were provided, use hardcoded points
            hardcoded_points = [
                (0.1345, 0.0735, 0.1359),  # home, back 10cm, rotate 30 deg
                (0.1345, -0.0735, 0.1359),  # home, back 10cm, rotate -30 deg
                (0.25, 0, 0.1714),  # home
                (0.25, 0, 0.1074),  # home, down
                (0.25, 0, 0.19),  # home, up

            ]  # m
            points_to_visit.extend(hardcoded_points)
        print(f"Saved Calibration Route => {points_to_visit}")
        return points_to_visit

    def shutdown(self):
        robot_shutdown()


if __name__ == '__main__':
    controller = Controller()
    controller.start()
