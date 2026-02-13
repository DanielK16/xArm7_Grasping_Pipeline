#!/usr/bin/env python3

# Publish the pointcloud into Rviz for visualization
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os
import time

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

class WorldFixedCloudPublisher(Node):
    def __init__(self):
        super().__init__('world_fixed_cloud_publisher')

        # File paths 
        self.color_path = "/home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/color.png"
        self.depth_path = "/home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/depth.png"

        # Frame settings
        self.world_frame = "link_base"
        self.cam_frame = "camera_color_optical_frame"

        # Camera Intrinsics 
        self.fx, self.fy = 912.0, 912.0
        self.cx, self.cy = 652.2, 377.4

        # Performance Settings
        self.decimation = 1 # 1 = full res, 4 = every 4th pixel. Reduces point count by 16x.

        # HEIGHT LIMITS (World Frame in Meters)
        # This filters points based on their Z-height AFTER transformation to the world frame.
        # Useful to remove the floor (z < 0) or ceiling points.
        self.world_z_min = 0.01  # Cut off everything below 1cm (e.g., the table surface itself)
        self.world_z_max = 1.0   # Cut off everything above 1 meter

        # TF2 Setup for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State tracking
        self.last_mtime = 0
        self.static_cloud_msg = None

        # Publisher
        self.pub = self.create_publisher(PointCloud2, '/static_scene_cloud', 10)

        # Timer: Check for file updates every 1.0s
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(f"[CloudPublisher] Active. Decimation: {self.decimation}x | Z-Limits: [{self.world_z_min}m, {self.world_z_max}m]")

    def get_cam_to_world_matrix(self):
        """
        Retrieves the 4x4 homogeneous transformation matrix from Camera to World frame.
        """
        try:
            # Look up transform at current time
            t = self.tf_buffer.lookup_transform(self.world_frame, self.cam_frame, rclpy.time.Time())

            # Convert Quaternion to Rotation Matrix
            quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            mat = np.eye(4)
            mat[:3, :3] = R.from_quat(quat).as_matrix()
            mat[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            return mat
        except Exception as e:
            self.get_logger().warn(f"TF Lookup failed: {e}. Is the robot state publisher running?")
            return None

    def generate_cloud(self):
        """
        Reads images, creates 3D points, transforms them to World Frame, and filters by height.
        """
        # Load Images
        if not os.path.exists(self.color_path) or not os.path.exists(self.depth_path):
            return None
            
        color = cv2.imread(self.color_path)
        depth = cv2.imread(self.depth_path, cv2.IMREAD_UNCHANGED)

        if color is None or depth is None:
            self.get_logger().warn("Failed to load images.")
            return None

        # Downsampling (Decimation)
        color = color[::self.decimation, ::self.decimation]
        depth = depth[::self.decimation, ::self.decimation]

        # Adjust intrinsics for downsampling
        fx, fy = self.fx / self.decimation, self.fy / self.decimation
        cx, cy = self.cx / self.decimation, self.cy / self.decimation

        # Convert BGR to RGB
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        h, w = depth.shape

        # Create Grid
        u, v = np.meshgrid(np.arange(w), np.arange(h))

        # Filter by Depth (Camera Frame)
        # Convert mm to meters
        z = depth.astype(float) / 1000.0
        # Basic depth mask: Ignore things too close (noise) or too far (background)
        mask = (z > 0.1) & (z < 1.5) 

        if not np.any(mask):
            return None

        # Project to 3D (Camera Frame)
        z_f = z[mask]
        x_c = (u[mask] - cx) * z_f / fx
        y_c = (v[mask] - cy) * z_f / fy

        # Create homogeneous coordinates [4 x N]
        points_cam = np.vstack((x_c, y_c, z_f, np.ones_like(z_f)))

        # Transform to World Frame
        t_matrix = self.get_cam_to_world_matrix()
        if t_matrix is None:
            return None

        # Apply transformation: Point_world = T * Point_cam
        points_world = t_matrix @ points_cam

        # HEIGHT FILTERING (World Frame)
        # points_world is [4 x N]. Row 2 is Z.
        z_world = points_world[2, :]
        
        # Create a mask for valid height based on config
        height_mask = (z_world >= self.world_z_min) & (z_world <= self.world_z_max)
        
        # Apply the mask to points
        points_world = points_world[:, height_mask]
        
        # Get raw colors corresponding to the depth mask
        r = color[mask][:, 0]
        g = color[mask][:, 1]
        b = color[mask][:, 2]
        
        # Apply the height filter to these colors
        r = r[height_mask].astype(np.uint32)
        g = g[height_mask].astype(np.uint32)
        b = b[height_mask].astype(np.uint32)

        if points_world.shape[1] == 0:
            self.get_logger().info("All points filtered out by height limit.")
            return None

        # Pack RGB for ROS (float32 view)
        rgb_packed = ((r << 16) | (g << 8) | b).view(np.uint32).view(np.float32)

        # Create PointCloud2 Message
        # Structure: [x, y, z, rgb]
        final_points = np.zeros((points_world.shape[1], 4), dtype=np.float32)
        final_points[:, 0] = points_world[0, :]
        final_points[:, 1] = points_world[1, :]
        final_points[:, 2] = points_world[2, :]
        final_points[:, 3] = rgb_packed

        header = Header()
        header.frame_id = self.world_frame
        # Use current time for stamp so RViz accepts it
        header.stamp = self.get_clock().now().to_msg() 

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        return point_cloud2.create_cloud(header, fields, final_points)

    def timer_callback(self):
        """Checks file timestamps and triggers processing if new data appears."""
        if os.path.exists(self.depth_path):
            try:
                current_mtime = os.path.getmtime(self.depth_path)
                # If file is newer than last read
                if current_mtime > self.last_mtime:
                    self.get_logger().info("New scene data detected. Processing...")
                    
                    # Sleep briefly to ensure TF buffer has latest robot state 
                    # (in case robot just stopped moving when picture was taken)
                    time.sleep(0.3)
                    
                    cloud = self.generate_cloud()
                    if cloud:
                        self.static_cloud_msg = cloud
                        self.last_mtime = current_mtime
                        self.get_logger().info(f"Cloud Published: {cloud.width} points.")
            except Exception as e:
                self.get_logger().error(f"Error accessing scene files: {e}")

        # Keep publishing the cached cloud so it doesn't disappear in RViz
        if self.static_cloud_msg:
            # Update timestamp to 'now' so RViz doesn't drop it as 'too old'
            self.static_cloud_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.static_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WorldFixedCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
