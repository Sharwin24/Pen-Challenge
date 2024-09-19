import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import sys


class Trackbar():
    def __init__(self, value_name, max_value, window_name, default=0) -> None:
        self.name = value_name
        self.max_value = max_value
        self.window_name = window_name
        self.value = default

    def create(self):
        cv2.createTrackbar(self.name, self.window_name,
                           self.value, self.max_value, self.handler)
        print(f"{self.name} Trackbar Created for {self.window_name}")

    def handler(self, val):
        # print(f"Handling {self.name} = {val}")
        self.value = val
        cv2.setTrackbarPos(self.name, self.window_name, val)

    def get(self):
        return cv2.getTrackbarPos(self.name, self.window_name)


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
        size = (1280, 720)  # (640, 480)  # Get from device type??
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
        self.df = aligned_depth_frame
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

    def get_centroid_pos(self, center):
        if center != None:
            dist_meters = self.df.get_distance(center[0], center[1])
            centroid_pos = rs.rs2_deproject_pixel_to_point(
                self.intr, center, dist_meters)
            return centroid_pos
        return None


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
    tuned_hsv = {'H_Low': 115,
                 'H_High': 154,
                 'S_Low': 85,
                 'S_High': 255,
                 'V_Low': 60,
                 'V_High': 255}
    low_hue = Trackbar("Low Hue", 179, "HSV Window", tuned_hsv["H_Low"])
    low_saturation = Trackbar("Low Saturation", 255,
                              "HSV Window", tuned_hsv["S_Low"])
    low_value = Trackbar("Low Value", 255, "HSV Window", tuned_hsv["V_Low"])
    high_hue = Trackbar("High Hue", 179, "HSV Window", tuned_hsv["H_High"])
    high_saturation = Trackbar(
        "High Saturation", 255, "HSV Window", tuned_hsv["S_High"])
    high_value = Trackbar("High Value", 255, "HSV Window", tuned_hsv["V_High"])
    trackbars = [low_hue, low_saturation, low_value,
                 high_hue, high_saturation, high_value]
    cv2.namedWindow("Smoothed Image Window", cv2.WINDOW_NORMAL)
    cv2.namedWindow("HSV Window", cv2.WINDOW_NORMAL)
    cv2.namedWindow("BG Removed Window", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Masked Window", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Contours Window", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Pen Contour Window", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Centroid Window", cv2.WINDOW_NORMAL)
    for tb in trackbars:
        tb.create()
    pipeline = Pipeline()
    try:
        while True:
            depth_image, color_image = pipeline.get_depth_and_color_images()
            # Apply bilateral filter to color image to smooth
            smooth = cv2.bilateralFilter(color_image, 15, 75, 75)
            cv2.imshow("Smoothed Image Window", smooth)
            ############# Begin_Citation [3] #################
            # Convert color image to HSV image and show in new window
            hsv_image = cv2.cvtColor(smooth, cv2.COLOR_BGR2HSV)
            test_hsv_mask = cv2.inRange(
                hsv_image,
                (low_hue.get(), low_saturation.get(), low_value.get()),
                (high_hue.get(), high_saturation.get(), high_value.get()),
            )
            tuned_hsv_mask = cv2.inRange(
                hsv_image,
                (tuned_hsv["H_Low"], tuned_hsv['S_Low'], tuned_hsv['V_Low']),
                (tuned_hsv["H_High"], tuned_hsv['S_High'],
                 tuned_hsv['V_High']),
            )
            ############# End_Citation [3] #################
            cv2.imshow("HSV Window", test_hsv_mask)
            # Cut off the pixels after a certain depth in the color image
            bg_removed = pipeline.remove_background(depth_image, smooth)
            cv2.imshow("BG Removed Window", bg_removed)
            ############# Begin_Citation [4] #################
            masked_without_bg = cv2.bitwise_and(
                bg_removed, smooth, mask=test_hsv_mask)
            masked_with_bg = cv2.bitwise_and(
                smooth, smooth, mask=test_hsv_mask)
            ############# End_Citation [4] #################
            cv2.imshow("Masked Window", masked_with_bg)
            # Now draw contours on the image and (hopefully) around the pen
            ############# Begin_Citation [7] #################
            grayscale = cv2.cvtColor(masked_with_bg, cv2.COLOR_BGR2GRAY)
            edged = cv2.Canny(grayscale, 30, 200)
            contours, hierarchy = cv2.findContours(
                edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.imshow("Contours Window", edged)
            cv2.drawContours(masked_with_bg, contours, -1, (0, 255, 0), 3)
            # Find the contour with the largest area
            center = None
            if len(contours) != 0:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(masked_with_bg, (x, y),
                              (x + w, y + h), (0, 255, 0), 2)
                center = (x + w // 2, y + h // 2)
            ############# End_Citation [7] #################
            cv2.imshow("Pen Contour Window", masked_with_bg)
            if center != None:
                ############# Begin_Citation [8] #################
                cv2.circle(smooth, center, 6, (0, 0, 255), -1)
                ############# End_Citation [8] ##################
                cv2.imshow("Centroid Window", smooth)
                pen_pos = pipeline.get_centroid_pos(center)
                if pen_pos != None:
                    print(f"Pen Position {pen_pos} m")
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
