
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from enum import Enum

import modern_robotics as mr
import numpy as np
import pickle
import matplotlib.pyplot as plt
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


def plot_points(self, robot_points, camera_points):
    robot_x = [rp[0] for rp in robot_points]
    robot_y = [rp[1] for rp in robot_points]
    camera_x = [cp[0] for cp in camera_points]
    camera_y = [cp[1] for cp in camera_points]
    rotated_x = []
    rotated_y = []
    # Robot = R * Camera + t
    R = self.params[0][0].as_matrix()
    for p in camera_points:
        new_robot_point = np.add(
            np.array(R) @ np.array(p), self.params[1])
        rotated_x.append(new_robot_point[0])
        rotated_y.append(new_robot_point[1])
    plt.plot(robot_x, robot_y, color='red', marker='o')
    plt.plot(camera_x, camera_y, color='green', marker='o')
    plt.plot(rotated_x, rotated_y, color='blue', marker='o')
    plt.show()


class CalibrationManager:

    ############# Begin_Citation [9] #############
    def save(self, file_name, calibration_points, computed_params):
        # Write to a file
        if os.path.exists(file_name):
            # If the file exists, acknowledge overwriting
            print(f"{file_name} already exists, overwriting!")
        with open(file_name, 'wb') as f:
            pickle.dump([calibration_points, computed_params], f)
        print(f"Saved {calibration_points}, {computed_params} to {file_name}")

    def load(self, file_name):
        # If file exists, load it
        if os.path.exists(file_name):
            with open(file_name, 'rb') as f:
                points, params = pickle.load(f)
            return points, params
        else:
            print(f"{file_name} does not exist")
    ############# End_Citation [9] #############


class PincherX100:
    class DIRECTION(Enum):
        UP = 0,
        DOWN = 1,
        FORWARD = 2,
        BACKWARD = 3,
        ROTATE = 4

    def __init__(self) -> None:
        np.set_printoptions(precision=2)
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")
        robot_startup()
        self.robot.gripper.set_pressure(0.25)
        self.joint_dict = {}
        self.update_joint_angle_dict()
        self.step_size = 5 / 100  # default is 5 cm, convert to meters
        self.calib_manager = CalibrationManager()
        self.visited_points = []

    def __repr__(self) -> str:
        return f"Waist: {np.rad2deg(self.joint_dict["waist"]):.2f}\N{DEGREE SIGN}" + \
            f"\nShoulder: {np.rad2deg(self.joint_dict["shoulder"]):.2f}\N{DEGREE SIGN}" + \
            f"\nElbow: {np.rad2deg(self.joint_dict["elbow"]): .2f}\N{DEGREE SIGN}" + \
            f"\nWrist Angle: {np.rad2deg(self.joint_dict["wrist_angle"]):.2f}\N{DEGREE SIGN}" + \
            f"\nEE: ({100 * self.ee_pos()[0]:.2f}, " + \
            f"{100 * self.ee_pos()[1]:.2f}, {100 * self.ee_pos()[2]:.2f}) cm"

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
        return np.array([T[0][3], T[1][3], T[2][3]])

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
            self.move(PincherX100.DIRECTION.UP, self.step_size)
        elif cmd == 'd' or cmd == 'down':
            self.move(PincherX100.DIRECTION.DOWN, self.step_size)
        elif cmd == 'f' or cmd == 'forward':
            self.move(PincherX100.DIRECTION.FORWARD, self.step_size)
        elif cmd == 'b' or cmd == 'backward':
            self.move(PincherX100.DIRECTION.BACKWARD, self.step_size)
        elif cmd == 'r' or cmd == 'rotate':
            angle_deg = input(
                f"Please enter the amount of degrees you want to rotate (absolute position): ")
            angle = float(angle_deg)
            self.move(self.DIRECTION.ROTATE, angle)
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
            self.create_calibration_points()
        elif cmd == 'v' or cmd == 'visiting':
            self.visit_points()
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
            case PincherX100.DIRECTION.UP:
                self.robot.arm.set_ee_pose_components(
                    current_ee_pos[0], current_ee_pos[1], current_ee_pos[2] + relative_distance)
            case PincherX100.DIRECTION.DOWN:
                self.robot.arm.set_ee_pose_components(
                    current_ee_pos[0], current_ee_pos[1], current_ee_pos[2] - relative_distance)
            case PincherX100.DIRECTION.FORWARD:
                self.robot.arm.set_ee_pose_components(
                    current_ee_pos[0] + relative_distance, current_ee_pos[1], current_ee_pos[2])
            case PincherX100.DIRECTION.BACKWARD:
                self.robot.arm.set_ee_pose_components(
                    current_ee_pos[0] - relative_distance,  current_ee_pos[1], current_ee_pos[2])
            case PincherX100.DIRECTION.ROTATE:
                return self.robot.arm.set_single_joint_position(
                    "waist", np.deg2rad(relative_distance))
            case _:
                print(f"Invalid movement direction given")
                return False

    def move_to_point(self, point):
        self.robot.arm.set_ee_pose_components(
            x=point[0], y=point[1], z=point[2], moving_time=1)

    def visit_points(self):
        # Visit points and if saving is desired, allow the user to save it
        visiting = True
        while visiting:
            point = input(
                f"Please input an absolute point [cm] as a tuple of (x,y,z) or [q] to quit visiting sequence: ")
            if point == 'q':
                print("Quitting Visiting loop")
                break
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

    def create_calibration_points(self):
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
        points_to_visit = []
        if len(self.visited_points) != 0:
            # If no points were loaded, and visiting loop was run, use those points
            print(f"Loading points from visiting loop:" +
                  f"\n{self.visited_points}")
            points_to_visit.extend(self.visited_points)
        elif len(self.visited_points) == 0:
            # If no points at all were provided, use hardcoded points
            hardcoded_points = [
                [0.64, -24.51, 8.6],  # Middle Low
                [-20.03, -14.81, 10.03],  # r(-245),2x down
                [-13.34, -20.57, 8.58],  # Right Low
                [0.43, -25.4, 18.46],  # Middle High
                [-24.90, -0.5, 10.76],  # All the way right
                [-9.03, -23.74, 18.46],  # Right High
            ]  # cm
            hardcoded_points = np.array(
                list(
                    map(lambda p: [p[0] / 100, p[1] / 100, p[2] / 100], hardcoded_points))
            )
            points_to_visit.extend(hardcoded_points)
        print(f"Saved Calibration Route => {points_to_visit}")
        return points_to_visit

    def compute_calibration_parameters(self, robot_points, camera_points, debug=False):
        # Find the centroid of both points
        N_r = len(robot_points)
        robot_centroid = np.array([
            sum([p[0] for p in robot_points]) / N_r,
            sum([p[1] for p in robot_points]) / N_r,
            sum([p[2] for p in robot_points]) / N_r,
        ])
        N_c = len(camera_points)
        camera_centroid = np.array([
            sum([p[0] for p in camera_points]) / N_c,
            sum([p[1] for p in camera_points]) / N_c,
            sum([p[2] for p in camera_points]) / N_c,
        ])
        subtracted_robot_centroids = np.array([
            [robot_centroid[0] - p[0],
             robot_centroid[1] - p[1],
             robot_centroid[2] - p[2]] for p in robot_points
        ])
        N_sr = len(subtracted_robot_centroids)
        subtracted_camera_centroids = np.array([
            [camera_centroid[0] - p[0],
             camera_centroid[1] - p[1],
             camera_centroid[2] - p[2]] for p in camera_points
        ])
        N_sc = len(subtracted_camera_centroids)
        if debug:
            print(
                f"====== Computation Debugging Information ======\n" +
                f"{N_r} Robot Points {robot_points}\n" +
                f"{N_c} Camera Points {camera_points}\n" +
                f"Robot Centroid {robot_centroid}\n" +
                f"Camera Centroid {camera_centroid}\n" +
                f"{N_sr} Subtracted Robot Points {subtracted_robot_centroids}\n" +
                f"{N_sc} Subtracted Camera Points {
                    subtracted_camera_centroids}"
            )
        R = tr.Rotation.align_vectors(
            subtracted_camera_centroids,
            subtracted_robot_centroids
        )[0].as_matrix()
        R_camera = R @ camera_centroid
        t = np.subtract(robot_centroid, R_camera)
        if debug:
            print(
                f"Rotation Matrix {R}\n" +
                f"Rotated Camera {R_camera}\n" +
                f"Translation Vector {t}\n" +
                f"===============================================\n"
            )
        return [R, t]

    def transform_point(self, camera_point, calib_params):
        R = calib_params[0]
        t = calib_params[1]
        return np.add(R @ camera_point, t)

    def shutdown(self):
        robot_shutdown()


if __name__ == '__main__':
    controller = PincherX100()
    controller.start()
