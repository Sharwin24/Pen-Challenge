from pipeline import Pipeline, Trackbar
from robot_control import Calibrator, Controller, DIRECTION
import time


def calibration_sequence(robot: Controller, camera: Pipeline):
    get_new_points = input("Get new calibration points? [y, n]: ")
    robot_points = []
    if get_new_points == 'y':
        robot.visiting()
        robot_points = robot.calibrate()
    else:
        robot_points = robot.calibrate()
    camera_points = []
    for p in robot_points:
        robot.move_to_point(p)
        camera.run_image_pipeline()
        camera_points.append(camera.get_pen_pos())
        print(f"Robot {p}\tCamera {camera_points[-1]}")
        time.sleep(2)
    robot.cb.compute_calibration_parameters(robot_points, camera_points)
    robot.cb.save("calibration.pkl")


def cleanup():
    robot.sleep()
    robot.shutdown()
    camera.stop()


if __name__ == '__main__':
    # Create objects
    robot = Controller()
    camera = Pipeline()
    robot.home()
    calibrate = input(f"Run calibration sequence? [y, n]: ")
    if calibrate == 'y':
        robot.sleep()
        calibration_sequence(robot, camera)
    else:
        robot.start()
    cleanup()
