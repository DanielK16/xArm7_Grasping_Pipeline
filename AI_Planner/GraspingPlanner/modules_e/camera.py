import pyrealsense2 as rs
import numpy as np

class RealSenseCam:
    def __init__(self, cfg):
        self.cfg = cfg
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)

        # Post Processing Filters
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, self.cfg.FILTER_SPATIAL_MAGNITUDE)
        self.spatial.set_option(rs.option.filter_smooth_alpha, self.cfg.FILTER_SPATIAL_ALPHA)
        self.spatial.set_option(rs.option.filter_smooth_delta, self.cfg.FILTER_SPATIAL_DELTA)
        self.temporal = rs.temporal_filter()
        self.temporal.set_option(rs.option.filter_smooth_alpha, self.cfg.FILTER_TEMPORAL_ALPHA)
        self.temporal.set_option(rs.option.filter_smooth_delta, self.cfg.FILTER_TEMPORAL_DELTA)
        self.hole_filling = rs.hole_filling_filter()
        self.hole_filling.set_option(rs.option.holes_fill, self.cfg.FILTER_HOLE_FILL_MODE)

    def start(self):
        print(">> [CAM] Starting Camera...")
        self.config.enable_stream(rs.stream.color, self.cfg.CAM_WIDTH, self.cfg.CAM_HEIGHT, rs.format.bgr8, self.cfg.CAM_FPS)
        self.config.enable_stream(rs.stream.depth, self.cfg.CAM_WIDTH, self.cfg.CAM_HEIGHT, rs.format.z16, self.cfg.CAM_FPS)
        
        profile = self.pipeline.start(self.config)
        
        # Increase Laser Power for better depth resolution
        depth_sensor = profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.laser_power):
            depth_sensor.set_option(rs.option.laser_power, self.cfg.LASER_POWER)

        print(">> [CAM] Camera Ready.")

    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        # apply pre processing filters at depth image
        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.hole_filling.process(depth_frame)

        # convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        return color_image, depth_image

    def stop(self):
        self.pipeline.stop()

    def get_camera_info(self, get_resolution = False):
        get_resolution = self.cfg.SHOW_RES
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        # Print all available resolutions
        if (get_resolution):
            # Print Product S/N, Name and USB Port
            print(device_product_line)
            for sensor in device.query_sensors():
                        print(f"\nSensor: {sensor.get_info(rs.camera_info.name)}")
                        profiles = sensor.get_stream_profiles()
                        for p in profiles:
                            print(f"  {p}")

