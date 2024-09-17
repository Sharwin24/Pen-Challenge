import pyrealsense2 as rs
import numpy as np
import cv2


class TrackbarSettings():
    pass


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
        self.clipping_distance = self.set_clipping_distance(clip_dist)

        # Create an align object
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def has_rgb_camera(self, device) -> bool:
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                return True
        return False

    def set_clipping_distance(self, distance_meters):
        self.clipping_distance = distance_meters / self.depth_scale

    def get_frames(self):
        return self.pipeline.wait_for_frames()

    def get_aligned_frames(self, frames):
        self.align.process(frames)

    def record_stream(self):
        pass

    def stop(self):
        self.pipeline.stop()

    def pipe(self):
        pass


image_pipeline = Pipeline()

try:
    while True:
        pass
finally:
    image_pipeline.stop()
