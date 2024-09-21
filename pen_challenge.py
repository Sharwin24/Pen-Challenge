from pipeline import Camera
from robot_control import PincherX100
import time


def calibration_sequence(robot: PincherX100, camera: Camera):
    robot_points = []
    robot.visit_points()
    robot_points = robot.create_calibration_points()
    camera_points = []
    for p in robot_points:
        robot.move_to_point(p)
        camera.run_image_pipeline()
        camera_points.append(camera.get_pen_pos())
        print(f"Robot {robot.ee_pos()} --> Camera " +
              f"({camera_points[-1][0]:.2f}, {camera_points[-1][1]:.2f}, {camera_points[-1][2]:.2f})")
        time.sleep(2)
    computed_params = robot.compute_calibration_parameters(
        robot_points, camera_points, debug=True)
    robot.calib_manager.save("calibration.pkl", robot_points, computed_params)
    # robot.cb.plot_points(robot_points, camera_points)
    return robot_points, computed_params


def robot_pursuit(robot, camera, calib_params):
    print(f"Running Robot Pursuit:\n" +
          f"R = {calib_params[0]}\nt= {calib_params[1]}")
    # While we are far enough from the point
    # Move towards the point
    # Once in the general area, slowly lower wrist and grab pen


if __name__ == '__main__':
    try:
        robot = PincherX100()
        camera = Camera()
        calibrate = input(f"Run new calibration sequence? [y, n]: ")
        if calibrate == 'y':
            robot.sleep()
            points, params = calibration_sequence(robot, camera)
        else:
            points, params = robot.calib_manager.load("calibration.pkl")
        print(f"Calibrated Robot with points {points}\n" +
              f"Rotation Matrix {params[0]} and Translation {params[1]}")
        # Run point pursuit
        robot_pursuit(robot, camera, params)
    finally:
        robot.sleep()
        robot.shutdown()
        camera.stop()
