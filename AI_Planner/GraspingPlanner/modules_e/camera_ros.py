import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters
import threading
import numpy as np
import time

class RealSenseCam:
    def __init__(self, cfg):
        self.cfg = cfg

        # Initialise CV Bridge
        self.bridge = CvBridge()
        
        # Initialise rclpy
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node('camera_subscriber_node')
        
        self.latest_color = None
        self.latest_depth = None
        self._lock = threading.Lock()
        
        # Subscriptions to realsense Images
        self.color_sub = message_filters.Subscriber(
            self.node, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self.node, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        # Synchronisiere Color and Depth Frames
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self._callback)

        # Start ROS Spinning in a separate thread
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)

    def _spin(self):
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f">> [ROS2] Spin interrupted: {e}")

    def _callback(self, color_msg, depth_msg):
        try:
            # Convert ROS-Messages in OpenCV/Numpy Format
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            
            with self._lock:
                self.latest_color = color_img
                self.latest_depth = depth_img
        except Exception as e:
            print(f">> [CAM ERROR] Callback conversion failed: {e}")

    def start(self):
        print(">> [CAM] Connecting to ROS 2 Topics...")
        if not self.spin_thread.is_alive():
            self.spin_thread.start()

        timeout = 10
        start_time = time.time()
        while self.latest_color is None:
            if (time.time() - start_time) > timeout:
                print("!! [CAM ERROR] Timeout: No ROS 2 topics found. Did you launch the RealSense node?")
                return
            time.sleep(0.5)
        print(">> [CAM] Successfully receiving ROS 2 frames.")

    def get_frames(self):
        with self._lock:
            if self.latest_color is None or self.latest_depth is None:
                return None, None
            return self.latest_color.copy(), self.latest_depth.copy()

    def stop(self):
        """Stoppping the node."""
        print(">> [CAM] Stopping Subscriber...")
        self.node.destroy_node()
        # rclpy.shutdown() # Optional, falls andere Nodes noch laufen

    def get_camera_info(self):
        print("--- Camera Interface: ROS 2 Subscriber ---")
        print("Subscribed to: /camera/color/image_raw")
        print("Subscribed to: /camera/aligned_depth_to_color/image_raw")
        print("------------------------------------------")