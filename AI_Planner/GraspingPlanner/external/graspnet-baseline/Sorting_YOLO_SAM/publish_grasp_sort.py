import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
import os
import argparse


parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=['raw', 'orientation_only', 'mask_only', 'orientation_mask'], default='raw', help='Choose grasp filtering mode')
args = parser.parse_args()


class StaticGraspFramePublisher(Node):
    def __init__(self, mode):
        super().__init__('static_grasp_frame_publisher')
        self.mode = mode
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.file_path = '/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/Scene_data/predicted_grasps.npy'
        self.last_mod_time = 0

        # Interval to check if predicted_grasps_matchappend.npy is changed on disk
        self.timer = self.create_timer(1.0, self.timer_callback)

    # Check if predicted_grasps_matchappend.npy is changed on disk
    def timer_callback(self):
        try:
            current_mod_time = os.path.getmtime(self.file_path)
            if current_mod_time != self.last_mod_time:
                self.last_mod_time = current_mod_time
                self.load_and_publish()
        except FileNotFoundError:
            self.get_logger().warn(f"File not found: {self.file_path}")

    def load_and_publish(self):
        try:
            grasps = np.load(self.file_path)
            if self.mode == 'raw':
                filtered_grasps = grasps
            elif self.mode == 'orientation_only':
                filtered_grasps = grasps[grasps[:, 17] == 1]
            elif self.mode == 'mask_only':
                filtered_grasps = grasps[grasps[:, 20] == 1]
            elif self.mode == 'orientation_mask':
                filtered_grasps = grasps[(grasps[:, 17] == 1) & (grasps[:, 20] == 1)]
            else:
                filtered_grasps = grasps

            if filtered_grasps.shape[0] == 0:
                self.get_logger().warn("No valid grasps found in file.")
                
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'camera_color_optical_frame'
                transform.child_frame_id = 'grasp_frame'
    
                transform.transform.translation.x = 0.0
                transform.transform.translation.y = 0.0
                transform.transform.translation.z = 0.0
    
                transform.transform.rotation.x = 0.0
                transform.transform.rotation.y = 0.0
                transform.transform.rotation.z = 0.0
                transform.transform.rotation.w = 1.0
    
                self.tf_static_broadcaster.sendTransform(transform)
                return

            top = filtered_grasps[0] # Select top grasps from filtered list of grasps
            translation = top[13:16] # Extract XYZ coordinates from filtered list of grasps
            rotation_matrix = top[4:13].reshape(3, 3) # Extract orientation from filtered list of grasps

            # apply axis swapping to orientation
            swap_matrix = np.array([
                [0, 0, 1],
                [1, 0, 0],
                [0, 1, 0]
            ])
            rotation_matrix = rotation_matrix @ swap_matrix

            quaternion = R.from_matrix(rotation_matrix).as_quat()

            # Publish grasp_frame as a child of camera_color_optical_frame
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'camera_color_optical_frame'
            transform.child_frame_id = 'grasp_frame'

            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])

            transform.transform.rotation.x = float(quaternion[0])
            transform.transform.rotation.y = float(quaternion[1])
            transform.transform.rotation.z = float(quaternion[2])
            transform.transform.rotation.w = float(quaternion[3])

            self.tf_static_broadcaster.sendTransform(transform)
            self.get_logger().info("Published updated static grasp_frame")
            # self.get_logger().info(f"x:  {float(translation[0])}")
            # self.get_logger().info(f"y:  {float(translation[1])}")
            # self.get_logger().info(f"z:  {float(translation[2])}")
            # self.get_logger().info(f"qx: {float(quaternion[0])}")
            # self.get_logger().info(f"qy: {float(quaternion[1])}")
            # self.get_logger().info(f"qz: {float(quaternion[2])}")
            # self.get_logger().info(f"qw: {float(quaternion[3])}")


        except Exception as e:
            self.get_logger().error(f"Failed to load or publish grasp data: {e}")

def main():
    rclpy.init()
    node = StaticGraspFramePublisher(mode=args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
