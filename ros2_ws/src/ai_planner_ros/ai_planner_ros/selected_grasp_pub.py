import rclpy
from rclpy.node import Node
import json
import os
import cv2
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class SelectedGraspPub(Node):
    def __init__(self):
        super().__init__('selected_grasp_pub')
        
        # Initialize publisher for PoseArray
        grasp_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # This fixes the DURABILITY_QOS_POLICY error
        )
        self.publisher = self.create_publisher(PoseArray, '/selected_grasp_pose', grasp_qos)

        # Initialize subscriber for camera intrinsics
        # Ensure the topic name matches your hardware setup
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info', 
            self.info_callback,
            10)

        # Internal state for camera parameters
        self.intrinsics_ready = False
        self.fx = self.fy = self.cx = self.cy = None
        self.depth_factor = 1000.0  # Conversion factor (mm to m)

        # File paths for scene data
        self.depth_img_path = "/home/rric/AI_Planner/GraspingPlanner/scene_data/depth.png"
        self.json_path = "/home/rric/AI_Planner/GraspingPlanner/scene_data/selected_grasp.json"

        # Execution timer (5 Hz / every 200ms)
        self.timer = self.create_timer(0.2, self.callback)
        self.get_logger().info("PoseArray Grasp Publisher initialized.")

    def info_callback(self, msg):
        """
        Extract intrinsic parameters from the CameraInfo message.
        K Matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        """
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        
        if not self.intrinsics_ready:
            self.get_logger().info(f"Intrinsics received: fx={self.fx}, fy={self.fy}")
            
        self.intrinsics_ready = True

    def _get_median_depth(self, depth_img, u, v):
        """
        Extract median depth from a 5x5 patch to filter out noise/outliers.
        """
        h, w = depth_img.shape
        u_m, v_m = int(u), int(v)
        
        # Extract patch while respecting image boundaries
        patch = depth_img[max(0, v_m-2):min(h, v_m+3), 
                          max(0, u_m-2):min(w, u_m+3)]
        
        # Filter out invalid depth values (0)
        valid = patch[patch > 0]
        return float(np.median(valid)) if valid.size > 0 else 0.0

    def _pixel_to_3d_simple(self, depth_img, point):
        """
        Project 2D pixel coordinates to 3D camera space using pinhole model.
        """
        u, v = point
        z_raw = self._get_median_depth(depth_img, u, v)
        
        # Convert raw depth (usually mm) to meters
        z = z_raw / self.depth_factor 
        
        if z <= 0:
            return None
            
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    def callback(self):
        """
        Main processing loop: loads data, computes 3D pose, and publishes PoseArray.
        """
        if not self.intrinsics_ready:
            self.get_logger().warn("Waiting for Intrinsics...") 
            return
    
        if not os.path.exists(self.json_path):
            self.get_logger().error(f"Missing JSON file at {self.json_path}")
            return
            
        if not os.path.exists(self.depth_img_path):
            self.get_logger().error(f"Missing Depth image at {self.depth_img_path}")
            return

        try:
            # 1. Load Pixel data
            with open(self.json_path, 'r') as f:
                json_data = json.load(f)

            # 2. Load Depth image
            depth_img = cv2.imread(self.depth_img_path, cv2.IMREAD_UNCHANGED)
            if depth_img is None:
                return

            u, v = json_data["center_pixel"]

            # 3. Perform 2D -> 3D projection
            coords = self._pixel_to_3d_simple(depth_img, (u, v))
            
            if coords:
                x, y, z = coords
                
                # --- PRINTING ALL PARAMETERS ---
                # This log helps you verify if the intrinsics from CameraInfo 
                # match the calculated 3D coordinates.
                self.get_logger().info(
                    f"\n[DEBUG DATA]\n"
                    f"Intrinsics -> fx: {self.fx:.2f}, fy: {self.fy:.2f}, cx: {self.cx:.2f}, cy: {self.cy:.2f}\n"
                    f"Projection -> x: {x:.4f}, y: {y:.4f}, z: {z:.4f}"
                )

                # 4. Construct PoseArray message
                array_msg = PoseArray()
                array_msg.header.stamp = self.get_clock().now().to_msg()
                array_msg.header.frame_id = "camera_color_optical_frame"

                p = Pose()
                p.position.x = float(x)
                p.position.y = float(y)
                p.position.z = float(z)
                p.orientation.w = 1.0  

                array_msg.poses.append(p)
                self.publisher.publish(array_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in processing callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SelectedGraspPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()