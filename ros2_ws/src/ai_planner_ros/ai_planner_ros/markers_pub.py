import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import json
import os
import numpy as np

# Marker Import 
from visualization_msgs.msg import Marker, MarkerArray

# TF2 Imports für die Transformation
from tf2_ros import Buffer, TransformListener

class GraspVisualizer(Node):
    def __init__(self):
        super().__init__('grasp_visualizer')
        
        # Create a publisher for the markers
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_grasps_markers', 10)
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Configuration Frames
        self.target_frame = "world"
        self.source_frame = "camera_color_optical_frame"
        
        # Paths to Json Grasps and Meshes of gripper
        self.json_path = '/home/rric/AI_Planner/GraspingPlanner/scene_data/grasp_points.json'
        self.mesh_dir = '/home/rric/ros2_ws/src/xarm_ros2/xarm_description/meshes/gripper/xarm'
        
        # Top Grasps Color
        self.grasp_colors = [
            (0.1, 0.8, 0.1),  # green top 1
            (0.1, 0.1, 0.8),  # blue top 2
            (0.8, 0.1, 0.1)   # red top 3
        ]
        self.last_mtime = 0

        self.timer = self.create_timer(0.1, self.publish_from_json)
        self.get_logger().info(f"Visualizer active. Transformed from {self.source_frame} to {self.target_frame}")

    def get_transform_matrix(self, x, y, z, roll=0, pitch=0, yaw=0):
        mat = np.eye(4)
        mat[0:3, 0:3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        mat[0:3, 3] = [x, y, z]
        return mat

    def create_mesh_marker(self, grasp_id, part_id, matrix, mesh_name, color, lifetime=0.0):
        marker = Marker()
        marker.header.frame_id = self.target_frame 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "xarm_gripper"
        marker.id = (grasp_id * 10) + part_id
        marker.lifetime = rclpy.duration.Duration(seconds=lifetime).to_msg()
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = f"file://{os.path.join(self.mesh_dir, mesh_name)}"
        
        pos = matrix[0:3, 3]
        quat = R.from_matrix(matrix[0:3, 0:3]).as_quat()
        
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = quat
        
        marker.scale.x, marker.scale.y, marker.scale.z = 1.0, 1.0, 1.0
        marker.color.a = 0.8
        marker.color.r, marker.color.g, marker.color.b = color
        return marker

    def publish_from_json(self):
        # Check if file exist
        if not os.path.exists(self.json_path):
            #print("Waiting until grasps exist")
            return

        #Check if grasps in this file are already published
        current_mtime = os.path.getmtime(self.json_path)
        if current_mtime <= self.last_mtime:
            return  

        # Get transform from target_frame to source_frame
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, now)
            
            # convert to 4x4 Matrix
            t = trans.transform.translation
            q = trans.transform.rotation
            # Create 4x4 Matrix with 1s in the diagonal
            T_world_camera = np.eye(4)
            # Save Rotation Matrix into first three collumns and rows
            T_world_camera[0:3, 0:3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            # Save Translation Matrix
            T_world_camera[0:3, 3] = [t.x, t.y, t.z]
            
        except Exception as e:
            self.get_logger().warn(f"Wait for tf connection {e}")
            return

        self.last_mtime = current_mtime

        # Open Json Path with Grasps
        try:
            with open(self.json_path, 'r') as f:
                grasps = json.load(f)
        except: return

        obj_label = grasps[0]["label"]

        # Create Marker Object
        marker_array = MarkerArray()
        
        # Choose best 3 grasps of json file and define ROS Pose
        # i = unique_id
        # q = data
        for i, g in enumerate(grasps[:3]):
            # Transform data grasps into a 4x4 Matrix
            grasp_rot = R.from_euler('xyz', [
                g['orientation_coords']['roll_deg'],
                g['orientation_coords']['pitch_deg'],
                g['orientation_coords']['yaw_deg']
            ], degrees=True).as_matrix()
            
            T_camera_tcp_raw = np.eye(4)
            T_camera_tcp_raw[0:3, 0:3] = grasp_rot
            T_camera_tcp_raw[0:3, 3] = [g['camera_coords']['x'], g['camera_coords']['y'], g['camera_coords']['z']]

            # Choose grasp color
            current_grasp_color = self.grasp_colors[i] if i < len(self.grasp_colors) else (0.5, 0.5, 0.5)

            # Correction of Matrix
            STRATEGY_GRASPNET = ['bowl_orange', 'bowl_green', 'cup', 'apple', 'bowl']
            STRATEGY_VERTICAL = ['spoon_orange', 'chopstick']
            if obj_label in STRATEGY_GRASPNET:
                rotation_correction = self.get_transform_matrix(0, 0, 0, roll=0, pitch=0, yaw=-np.pi/2)
            elif obj_label in STRATEGY_VERTICAL:
                rotation_correction = self.get_transform_matrix(0, 0, 0, roll=np.pi, pitch=0, yaw=0)

            # Multiplication on right side to turn around own local axis
            # If multiplication on left side we turn complete coordination system around!
            T_camera_tcp = T_camera_tcp_raw @ rotation_correction

            # Transform grasp frame to world frame
            # T_world_tcp = T_world_camera * T_camera_tcp
            T_world_tcp = T_world_camera @ T_camera_tcp

            # 4. Offset to base link
            base_offset = self.get_transform_matrix(0, 0, -0.172)
            T_world_base = T_world_tcp @ base_offset

            # 5. Gripper Meshes (filename, id, offset_x, offset_y, offset_z, color)
            # defined in gripper_urdf xarm_gripper
            gripper_parts = [
                ("base_link.stl", 0, 0, 0, 0, (1.0, 1.0, 1.0)),
                ("left_outer_knuckle.stl", 1, 0, 0.035, 0.059098, current_grasp_color),
                ("left_inner_knuckle.stl", 2, 0, 0.02, 0.074098, current_grasp_color),
                ("left_finger.stl", 3, 0, 0.070465, 0.101137, current_grasp_color),
                ("right_outer_knuckle.stl", 4, 0, -0.035, 0.059098, current_grasp_color),
                ("right_inner_knuckle.stl", 5, 0, -0.02, 0.074098, current_grasp_color),
                ("right_finger.stl", 6, 0, -0.070465, 0.101137, current_grasp_color)
            ]

            for name, p_id, ox, oy, oz, col in gripper_parts:
                part_offset = self.get_transform_matrix(ox, oy, oz)
                # Jedes Teil relativ zur berechneten Welt-Basis-Pose
                final_part_matrix = T_world_base @ part_offset
                marker = self.create_mesh_marker(i, p_id, final_part_matrix, name, col, 25)
                marker_array.markers.append(marker)

        # Publish Marker msg
        self.marker_pub.publish(marker_array)
        print("Published New grasps for 25 seconds")

def main():
    rclpy.init()
    node = GraspVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
