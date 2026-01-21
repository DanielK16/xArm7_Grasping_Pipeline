import os
import sys
import json
import time
import numpy as np
from xarm.wrapper import XArmAPI
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
import tf_transformations
from geometry_msgs.msg import TransformStamped
import cv2
import json


class CoordinateTransformerAndArmMover(Node):
    def __init__(self):
        super().__init__('coordinate_transformer_and_arm_mover')

        #Properties Classifikation
        with open("/home/rric/GRASP/graspnet-baseline/scene_data/grasp_info.json", "r") as f:
            data = json.load(f)

        self.area_pixels = data["area_pixels"]
        self.angle_deg = data["angle_deg"]
        self.grasp_side=data["grasp_side"]
        self.is_bowl=data["is_bowl"]
        self.is_apple=data["is_apple"]
        self.is_trash=data["is_trash"]

        #Drop Off Points:
        if self.is_bowl:
            self.drop_pose =[100, -17.1, -0.5, 49.1, 0.7, 66.2, -1]   # cup drop
        elif self.is_apple:
            self.drop_pose= [141.5, 8.2, -6.8, 76.6, 2.1, 68.4, -13.9]    #apple drop
        elif self.is_trash:
            self.drop_pose = [0.3, -28.4, -0.3, 39.9, 0.7, 69.3, 0.6]  # everything else drop

        # Initialize the xArm
        self.ip = '192.168.1.223'
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_enable(True)

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.attempts = 0
        self.max_attempts = 10

        # Timer runs every 1 sec
        self.timer = self.create_timer(1.0, self.try_to_move_arm)

    def try_to_move_arm(self):
        self.attempts += 1
        self.get_logger().info(f"Attempt {self.attempts}: Trying to get transform from 'grasp_frame' to 'link_base'...")

        try:
            transform = self.tf_buffer.lookup_transform('link_base', 'grasp_frame', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform not available yet: {e}")
            if self.attempts >= self.max_attempts:
                self.get_logger().error("Max attempts reached. Exiting node.")
                self.cleanup_and_shutdown()
            return

        # Translation grasp_frame - base_link
        t = transform.transform.translation
        x, y, z = t.x, t.y, t.z

        #offset z from gripper to grap in middle 
        z=z-0.02

        # Rotation
        q = transform.transform.rotation
        quat = (q.x, q.y, q.z, q.w)

        # Correction: rotate 90 degrees around Z axis
        correction_quat = tf_transformations.quaternion_from_euler(0, 0, -(np.pi / 2))
        corrected_quat = tf_transformations.quaternion_multiply(quat, correction_quat)
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(corrected_quat)

        # Compute pre-grasp position
        rot_matrix = tf_transformations.quaternion_matrix(quat)[:3, :3]
        normal = -rot_matrix[:, 2]
        offset = 0.04
        pre_x = x + normal[0] * offset
        pre_y = y + normal[1] * offset
        pre_z = z + normal[2] * offset

        self.get_logger().info("Transform OK. Moving to pre-grasp and grasp positions...")
        self.move_arm_to_position(pre_x, pre_y, pre_z, x, y, z, roll, pitch, yaw)

        self.get_logger().info("Grasp attempt finished. Shutting down...")
        self.timer.cancel()
        #self.cleanup_and_shutdown()
        rclpy.get_default_context().call_soon_threadsafe(self.cleanup_and_shutdown)

    def move_arm_to_position(self, pre_x, pre_y, pre_z, grasp_x, grasp_y, grasp_z, roll, pitch, yaw):
        #Going to Capture Scene
        self.arm.set_servo_angle(angle=[0.3, -28.4, -0.3, 39.9, 0.7, 69.3, 0.6], speed=20, is_radian=False, wait=True)

        # Convert to mm
        pre_x_mm, pre_y_mm, pre_z_mm = pre_x * 1000, pre_y * 1000, pre_z * -1000
        grasp_x_mm, grasp_y_mm, grasp_z_mm = grasp_x * 1000, grasp_y * 1000, grasp_z * 1000

        safety_z=-146       
        # safety_z in mm
        if pre_z_mm < safety_z or grasp_z_mm < safety_z:
            print("\n Z axis safety limit reached: aborting movement!\n")
            #self.arm.disconnect() # optional, wenn du das Programm beenden willst
        else:
        #safe to move
            self.arm.set_gripper_speed(1500)
            self.arm.set_gripper_position(850, wait=True)
            time.sleep(0.1)
            self.arm.set_gripper_position(850, wait=True)

            self.get_logger().info(f"Moving to pre-grasp: x={pre_x_mm:.1f}, y={pre_y_mm:.1f}, z={pre_z_mm:.1f}")
            self.arm.set_position(pre_x_mm, pre_y_mm, pre_z_mm, roll=roll, pitch=pitch, yaw=yaw, speed=150, wait=True, is_radian=True)

            self.get_logger().info(f"Moving to grasp: x={grasp_x_mm:.1f}, y={grasp_y_mm:.1f}, z={grasp_z_mm:.1f}")
            self.arm.set_position(grasp_x_mm, grasp_y_mm, grasp_z_mm, roll=roll, pitch=pitch, yaw=yaw, speed=50, wait=True, is_radian=True)
            time.sleep(0.25)

            self.get_logger().info("Activating gripper")
            self.arm.set_gripper_position(-10, wait=True)
            time.sleep(0.5)

            self.get_logger().info("Returning to home")
            self.arm.set_servo_angle(angle=[0.3, -28.4, -0.3, 39.9, 0.7, 69.3, 0.6], speed=23, is_radian=False, wait=True)

            #if cup tilt to remove leftovers!
            if self.is_bowl:
                if self.grasp_side:
                    self.arm.set_servo_angle(angle=[0.3, -28.4, -0.3, 39.9, 0.7, 69.3, 180], speed=23, is_radian=False, wait=True)
                    self.arm.set_servo_angle(angle=[-15.9, -15.6, -11.5, 43.4, 35.5, 83.7, 141.7], speed=23, is_radian=False, wait=True)
                    self.arm.set_servo_angle(angle=[0.3, -28.4, -0.3, 39.9, 0.7, 69.3, 0.6], speed=23, is_radian=False, wait=True)
                else:
                    self.arm.set_servo_angle(angle=[1.4, -11.5, -35.8, 38.3, 37.9, 85.1, -39.4], speed=23, is_radian=False, wait=True)
                    self.arm.set_servo_angle(angle=[0.3, -28.4, -0.3, 39.9, 0.7, 69.3, 0.6], speed=23, is_radian=False, wait=True)
            
            self.get_logger().info("Going to box")
            self.arm.set_servo_angle(angle=self.drop_pose, speed=100, is_radian=False, wait=True)
            
            self.get_logger().info("Releasing gripper")
            self.arm.set_gripper_position(850, wait=True)
            time.sleep(0.25)

            self.get_logger().info("Returning to home")
            self.arm.set_servo_angle(angle=[0.3, -28.4, -0.3, 39.9, 0.7, 69.3, 0.6], speed=100, is_radian=False, wait=True)

            self.arm.disconnect()

    def cleanup_and_shutdown(self):
        self.get_logger().info("Cleaning up and shutting down ROS node...")
        self.timer.cancel()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)    

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformerAndArmMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Cleaning up...")
        node.cleanup_and_shutdown()


if __name__ == '__main__':
    main()
