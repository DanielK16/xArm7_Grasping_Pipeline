import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as R
import json
import os
import numpy as np

class MoveItGraspPublisherRaw(Node):
    def __init__(self):
        super().__init__('moveit_grasp_publisher_raw')
        
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        
        self.pose_array_pub = self.create_publisher(PoseArray, '/moveit_grasp_candidates', qos_profile)
        
        # path and frames
        self.json_path = '/home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/grasp_points.json'
        self.camera_frame = "camera_color_optical_frame"
        
        self.last_mtime = 0
        self.current_pose_array = None

        # Timer: Check for new file
        self.create_timer(0.1, self.check_file_update)
        
        # Timer: Publish Poses everz 500ms
        self.create_timer(0.2, self.publish_current_data)
        
        self.get_logger().info(f"Grasp Publisher active. Frame: {self.camera_frame}")

    def check_file_update(self):
        """ Check if JSON File is updated """
        if not os.path.exists(self.json_path):
            return

        try:
            current_mtime = os.path.getmtime(self.json_path)
            if current_mtime > self.last_mtime:
                self.get_logger().info("New Grasp data recevied. Loading JSON...")
                self.load_json_data()
                self.last_mtime = current_mtime
        except Exception as e:
            self.get_logger().error(f"Data Error: {e}")

    def load_json_data(self):
        """ Read Json File and save grasp as PoseArray """
        try:
            with open(self.json_path, 'r') as f:
                grasps_data = json.load(f)

            new_pose_array = PoseArray()
            new_pose_array.header.frame_id = self.camera_frame

            if not grasps_data:
                self.get_logger().warn("JSON ist leer.")
                self.current_pose_array = None
                return

            for g in grasps_data:
                p = Pose()
                p.position.x = float(g['camera_coords']['x'])
                p.position.y = float(g['camera_coords']['y'])
                p.position.z = float(g['camera_coords']['z'])
                
                r = R.from_euler('xyz', [
                    g['orientation_coords']['roll_deg'],
                    g['orientation_coords']['pitch_deg'],
                    g['orientation_coords']['yaw_deg']
                ], degrees=True)
                
                quat = r.as_quat()
                p.orientation.x = quat[0]
                p.orientation.y = quat[1]
                p.orientation.z = quat[2]
                p.orientation.w = quat[3]
                
                new_pose_array.poses.append(p)

            self.current_pose_array = new_pose_array
            self.get_logger().info(f"{len(new_pose_array.poses)} Loaded Poses.")

        except Exception as e:
            self.get_logger().error(f"JSON Error: {e}")

    def publish_current_data(self):
        """ Publish data """
        if self.current_pose_array is not None:
            # time stamp
            self.current_pose_array.header.stamp = self.get_clock().now().to_msg()
            self.pose_array_pub.publish(self.current_pose_array)

def main():
    rclpy.init()
    node = MoveItGraspPublisherRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
