import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import sys


class Trackbar():
    def __init__(self, value_name, min_value, max_value, window_name) -> None:
        self.name = value_name
        self.min_value = min_value
        self.max_value = max_value
        self.window_name = window_name

    def create(self):
        cv2.createTrackbar(self.name, self.window_name,
                           self.min_value, self.max_value, self.handler)
        print(f"{self.name} Trackbar Created for {self.window_name}")

    def handler(self, val):
        cv2.setTrackbarPos(self.name, self.window_name, val)

    def get(self):
        cv2.getTrackbarPos(self.name, self.window_name)


class Pipeline():
    def __init__(self, clip_dist: float = 1) -> None:
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        if not self.has_rgb_camera(device):
            print("Requires a Depth camera with Color sensor")
            exit(0)

        # Enable depth and color streams
        size = (640, 480)  # Get from device type??
        self.config.enable_stream(
            rs.stream.depth,
            size[0],
            size[1],
            rs.format.z16,
            30)
        self.config.enable_stream(
            rs.stream.color,
            size[0],
            size[1],
            rs.format.bgr8,
            30)
        profile = self.pipeline.start(self.config)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print(f"Depth Scale: {self.depth_scale}")
        # Default clipping distance to one meter
        self.clipping_distance = clip_dist / self.depth_scale

        # Get intrinsic parameters
        stream = profile.get_stream(rs.stream.color)
        self.intr = stream.as_video_stream_profile().get_intrinsics()

        # Create an align object
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def has_rgb_camera(self, device) -> bool:
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                return True
        return False

    def get_depth_and_color_images(self):
        frames = self.pipeline.wait_for_frames()
        # Align the depth frame to the color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return depth_image, color_image

    def remove_background(self, depth_image, color_image, bg_color=153):
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        bg_removed = np.where((depth_image_3d > self.clipping_distance) |
                              (depth_image_3d <= 0), bg_color, color_image)
        return bg_removed

    def render_depth_image(self, depth_image, bg_removed):
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            depth_image, alpha=0.03), cv2.COLORMAP_JET)
        image = np.hstack((bg_removed, depth_colormap))
        return image

    def record(self, file_name):
        self.config.enable_record_to_file(file_name)

    def play(self, file_name, repeat=True):
        self.config.enable_device_from_file(file_name, repeat_playback=repeat)

    def stop(self):
        self.pipeline.stop()

    def pipe(self):
        pass


if __name__ == '__main__':
    ############# Begin_Citation [2] #################
    parser = argparse.ArgumentParser(
        prog="Pen Recognizer",
        description="Process images to recognize a pen")
    parser.add_argument(
        "-r", '--record', action='store_true',
        help="record frames to a file")
    parser.add_argument("-f", "--filename", help="File to record to")
    args = parser.parse_args()
    ############# End_Citation [2] #################
    print(args)
    low_hue = Trackbar("Low Hue", 0, 179, "HSV Window")
    low_saturation = Trackbar("Low Saturation", 0, 255, "HSV Window")
    low_value = Trackbar("Low Value", 0, 255, "HSV Window")
    high_hue = Trackbar("High Hue", 0, 179, "HSV Window")
    high_saturation = Trackbar("High Saturation", 0, 255, "HSV Window")
    high_value = Trackbar("High Value", 0, 255, "HSV Window")
    trackbars = [low_hue, low_saturation, low_value,
                 high_hue, high_saturation, high_value]
    pipeline = Pipeline()
    try:
        while True:
            depth_image, color_image = pipeline.get_depth_and_color_images()
            ############# Begin_Citation [3] #################
            # Convert color image to HSV image and show in new window

            cv2.namedWindow("HSV Window", cv2.WINDOW_NORMAL)
            for tb in trackbars:
                tb.create()
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(
                hsv_image,
                (low_hue.get(), low_saturation.get(), low_value.get()),
                (high_hue.get(), high_saturation.get(), high_value.get()),
            )
            cv2.imshow("HSV Window", mask)
            key = cv2.waitKey(1)
            ############# End_Citation [3] #################
            # bg_removed = pipeline.remove_background(depth_image, color_image)
            # images = pipeline.render_depth_image(depth_image, bg_removed)
            # key = pipeline.create_window(images)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
