#!/home/rric/miniforge3/envs/AI_Planner/bin/python3

"""
Centers the tray to get a view
Subscribes to:
/camera/camera/color/image_raw
Publishes to:
/xarm/vc_set_cartesian_velocitiy
"""

import rclpy
from rclpy.node import Node
import time
# realsense ros sdk
from sensor_msgs.msg import Image
from xarm_msgs.msg import MoveVelocity
from xarm_msgs.srv import SetInt16
from std_srvs.srv import Trigger
from xarm_msgs.srv import MoveCartesian
from xarm_msgs.msg import RobotMsg
import numpy as np
# opencv
import cv2
from cv_bridge import CvBridge
# YOLO World
from ultralytics import YOLOWorld

class Calibrate_Tray(Node):
    def __init__(self):
        super().__init__('calibrate_tray')

        # Subscribe to RS RGB Image
        try:
            rgb_img_sub_topic = '/camera/camera/color/image_raw'
            self.rgb_img_sub = self.create_subscription(
                Image,
                rgb_img_sub_topic ,
                self.camera_callback,
                1   
            )  
            print(f"[Calibration] Image subscribed to topic: {self.rgb_img_sub.topic} ")
        except Exception as e:
            print(f"Subscribing to topic {rgb_img_sub_topic} failed") 

        # Create Publisher for Velocity Commands (x,y,z)
        try:
            vel_publisher_topic = '/xarm/vc_set_cartesian_velocity'
            self.vel_publisher = self.create_publisher(
                MoveVelocity,
                vel_publisher_topic,
                10
            )
            print(f"[Calibration] Velocity publishes to topic: {vel_publisher_topic} ")
        except Exception as e:
            print(f"Publishing to topic {vel_publisher_topic} failed") 

        # Initialize CV Bridge
        self.bridge = CvBridge()
        print("[Calibration] CV Bridge Initialized")

        # Intitialize YOLO World
        self.yolo_inference(yolo_path = "/home/rric/AI_Planner/GraspingPlanner/models/yolov8x-worldv2.pt")

        # Setup Robot (Move to Home Pos to identify tray)
        self.setup_robot()

        # Controller parameter
        self.kp = 0.5
        # if  error is smaller than x pixels to reduce jitter
        self.deadband = 10
        # max velocity
        self.max_v = 20.0

    def yolo_inference(self, yolo_path):
        try:
            # Load model
            self.model = YOLOWorld(yolo_path)
            print("[Calibration] Yolo Model loaded")
        except Exception as e:
            print(f"Exception raised: {e}")

        # Define Custom Classes
        self.prompts = [
            "dark brown plastic cafeteria tray",           
            "rectangular tray with textured surface",    
            "brown tray with raised edges and handles", 
            "the flat base of a brown serving tray",         
            "dark plastic tray on a light wooden table",
            "dinning table",
        ]
        self.model.set_classes(self.prompts)
        
        self.prompt_to_label = {
            "dark brown plastic cafeteria tray": "tray",           
            "rectangular tray with textured surface": "tray",     
            "brown tray with raised edges and handles": "tray",    
            "the flat base of a brown serving tray": "tray",        
            "dark plastic tray on a light wooden table": "tray",
            "dinning table": "Ignore",
        }

    def setup_robot(self):
        """
        Activates Services and moves robot to Home Position to start
        """
        self.get_logger().info("Setting Up Robot")
        # Activate Services
        mode_cli = self.create_client(SetInt16, '/xarm/set_mode')
        state_cli = self.create_client(SetInt16, '/xarm/set_state')
        clean_cli = self.create_client(Trigger, '/xarm/clean_error')
        enable_cli = self.create_client(SetInt16, '/xarm/motion_enable')
        set_pos_cli = self.create_client(MoveCartesian, '/xarm/set_position')
        
        # Set Clean Error
        self.get_logger().info("Resetting Errors")
        clean_cli.call_async(Trigger.Request())
        time.sleep(0.5)

        # Set Motion Enable
        self.get_logger().info("Enable Motion")
        req = SetInt16.Request()
        req.data = 1
        enable_cli.call_async(req)
        time.sleep(0.5)

        # Set Mode to 0
        self.get_logger().info("Set Mode: 0")
        req_mode = SetInt16.Request()
        req_mode.data = 0
        mode_cli.call_async(req_mode)
        time.sleep(0.5)
        #Set state to 0
        self.get_logger().info("Set State: 0")
        req_state = SetInt16.Request()
        req_state.data = 0 
        state_cli.call_async(req_state)
        time.sleep(0.5)
        # Move Home Position (Mode: 0)
        self.get_logger().info("Move to Home Position")
        home_req = MoveCartesian.Request()
        home_req.pose = [341.0, 0.0, 487.0, 3.1415, 0.0, 0.0] 
        home_req.speed = 50.0
        home_req.acc = 500.0
        home_req.wait = True
        set_pos_cli.call_async(home_req)
        time.sleep(6.0)

        # Set Mode to 5
        self.get_logger().info("Set Mode: 5")
        req.data = 5
        mode_cli.call_async(req)
        time.sleep(0.5)

        # Set State to 0
        self.get_logger().info("Set State: 0")
        req.data = 0
        state_cli.call_async(req)
        time.sleep(0.5)

    def camera_callback(self,img_msg):
        #self.get_logger().info('recevied a new image from frame_id %s' %msg.header.frame_id)
        
        # Convert ROS Image to OpenCV image
        color_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        
        # Yolo Inference to detect tray
        results = self.model.predict(
            source=color_img,
            conf=0.1,
            #iou=0.2, 
            imgsz=640,
            device='cuda:0',
            half=True,
            verbose=False,
            augment=False,
            agnostic_nms=True
        )

        # Map labels to tray
        #Take all Bounding Boxes
        res = results[0]
        # Iterate through all Bounding Boxes
        tray_candidates = []
        for box in res.boxes:
            cls_idx = int(box.cls[0].item())
            prompt = self.prompts[cls_idx]
            if self.prompt_to_label.get(prompt) == "tray":
                tray_candidates.append(box)

        target_box = None
        tray_found = False
        # select the tray candidate with the highest confidence score
        if tray_candidates:
            target_box = max(tray_candidates, key=lambda b: b.conf[0].item())
            tray_found = True
        if tray_found:  
            res.boxes = [target_box] 
            res.names = {int(target_box.cls[0].item()): "tray"}
            annotated_frame = res.plot()
        else:
            annotated_frame = color_img.copy()  

        # P Controller 
        if tray_found:
            coords = target_box.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = coords
            
            print(f"Coordinates of Tray: x1={x1}, y1={y1}, x2={x2}, y2={y2}")

            # Compute Center of Bounding Box
            center_bbox_x = int((x1 + x2) / 2)
            center_bbox_y = int((y1 + y2) / 2)

            # Compute Width of bounding box
            ist_width = abs(x1-x2)

            # Draw Circle for Visualization
            # BBox Center
            annotated_frame = cv2.circle(annotated_frame, center = (center_bbox_x, center_bbox_y), radius = 4, color=(0, 255, 0), thickness = 2)
            # Center Image
            annotated_frame = cv2.circle(annotated_frame,center=(640,360), radius=10, color=(0,0,255), thickness=2)

            # SOLL: CENTER_X, CENTER_Y SOLL_WIDTH
            center_img_x = 640
            center_img_y = 360
            soll_width = 930

            vel_x = 0.0
            vel_y = 0.0
            vel_z = 0.0

            # Compute Error for x and y and z
            error_x = center_img_x - center_bbox_x
            error_y = center_img_y - center_bbox_y
            error_z = soll_width - ist_width
        
            if abs(error_x) > self.deadband:
                vel_x = error_x *self.kp
            if abs(error_y) > self.deadband:
                vel_y = error_y * self.kp
            if abs(error_z) > self.deadband:
                vel_z = error_z * self.kp

            # Map robot velocitiy
            vel_x = error_y * self.kp
            vel_y = error_x * self.kp
            vel_z = error_z * self.kp*(-1.0)

            # Clip Max Velocity 
            vel_x = np.clip(vel_x, -self.max_v, self.max_v)
            vel_y = np.clip(vel_y, -self.max_v, self.max_v)
            vel_z = np.clip(vel_z, -self.max_v, self.max_v)

            print(f"vel_x = {vel_x}")
            print(f"vel_y = {vel_y}")
            print(f"vel_z = {vel_z}")

            if all(abs(e) <= 1 for e in [error_x, error_y, error_z]):
                cv2.putText(annotated_frame, "Target reached", (400, 440), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                print("[Calibration successfull]")
                raise SystemExit

            # Visual Information
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            color = (255, 255, 255) 
            thickness = 2

            cv2.putText(annotated_frame, f"VX: {vel_x:.2f} mm/s", (10, 30), font, font_scale, color, thickness)
            cv2.putText(annotated_frame, f"VY: {vel_y:.2f} mm/s", (10, 60), font, font_scale, color, thickness)
            cv2.putText(annotated_frame, f"VZ: {vel_z:.2f} mm/s", (10, 90), font, font_scale, color, thickness)
            cv2.putText(annotated_frame, f"Error X in px: {error_x}", (10, 120), font, font_scale, (0, 255, 255), thickness)
            cv2.putText(annotated_frame, f"Error Y in px: {error_y}", (10, 150), font, font_scale, (0, 255, 255), thickness)
            cv2.putText(annotated_frame, f"Error Z in px: {error_z}", (10, 180), font, font_scale, (0, 255, 255), thickness)
            cv2.putText(annotated_frame, f"K_p: {self.kp:.2f}", (10, 210), font, font_scale, color, thickness)
            cv2.putText(annotated_frame, f"Threshold in px: {self.deadband:.2f} ", (10, 240), font, font_scale, color, thickness)
            
            # Publish velocity msg:
            vel_msg = MoveVelocity()
            vel_msg.speeds = [vel_x, vel_y, vel_z, 0.0, 0.0 , 0.0]
            vel_msg.is_sync = True
            vel_msg.is_tool_coord = False
            vel_msg.duration = 0.2
            self.vel_publisher.publish(vel_msg)
        
        else:
            stop_msg = MoveVelocity()
            stop_msg.speeds = [0.0] * 6
            self.vel_publisher.publish(stop_msg)

        # Stream Image
        cv2.imshow("Calibrate Tray", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Calibrate_Tray()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info("Node wird beendet...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
