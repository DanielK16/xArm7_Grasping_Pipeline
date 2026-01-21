import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from xarm_msgs.msg import MoveVelocity
from xarm_msgs.srv import SetInt16
from std_srvs.srv import Trigger
import numpy as np
import cv2
import time
from cv_bridge import CvBridge
from ultralytics import YOLOWorld

class Calibrate_Tray(Node):
    def __init__(self):
        super().__init__('calibrate_tray')

        # 1. Kommunikation Setup
        self.vel_publisher = self.create_publisher(MoveVelocity, '/xarm/vc_set_cartesian_velocity', 10)
        self.bridge = CvBridge()
        
        # 2. Roboter Hardware "Wachschütteln"
        self.init_robot_hardware()

        # 3. Vision Setup
        self.rgb_img_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback, 1)
        
        yolo_path = "/home/rric/AI_Planner/GraspingPlanner/models/yolov8x-worldv2.pt"
        self.model = YOLOWorld(yolo_path)
        self.model.set_classes(["brown rectangle tray"])

        # 4. Regelparameter
        self.kp = 0.05
        self.deadband = 15
        self.max_v = 40.0 

        self.get_logger().info("Setup vollständig abgeschlossen.")

    def call_service_sync(self, client, request, name):
        """ Hilfsfunktion: Wartet bis ein Service wirklich geantwortet hat. """
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Warte auf Service {name}...')
        
        future = client.call_async(request)
        # Wir nutzen rclpy.spin_until_future_complete nicht im Init eines Nodes, 
        # daher nutzen wir eine kleine Warteschleife für asynchrone Calls im Konstruktor
        return future

    def init_robot_hardware(self):
        self.get_logger().info("Initialisiere xArm7 Hardware...")
        
        # Clients
        clean_cli = self.create_client(Trigger, '/xarm/clean_error')
        enable_cli = self.create_client(SetInt16, '/xarm/motion_enable')
        mode_cli = self.create_client(SetInt16, '/xarm/set_mode')
        state_cli = self.create_client(SetInt16, '/xarm/set_state')

        # Sequenz ausführen (Wichtig: Pausen zwischen den Befehlen!)
        self.get_logger().info("1. Clean Error...")
        clean_cli.call_async(Trigger.Request())
        time.sleep(1.0) 

        self.get_logger().info("2. Motion Enable...")
        req = SetInt16.Request()
        req.data = 1
        enable_cli.call_async(req)
        time.sleep(1.0)

        self.get_logger().info("3. Set Mode 5 (Velocity)...")
        req.data = 5
        mode_cli.call_async(req)
        time.sleep(1.0)

        self.get_logger().info("4. Set State 0 (Ready)...")
        req.data = 0
        state_cli.call_async(req)
        time.sleep(1.0)
        
        self.get_logger().info("Hardware-Initialisierung beendet.")

    def camera_callback(self, msg):
        color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w, _ = color_img.shape
        c_img_x, c_img_y = w // 2, h // 2

        results = self.model.predict(source=color_img, conf=0.1, imgsz=640, device='cuda:0', verbose=False)
        annotated_frame = results[0].plot()

        vel_msg = MoveVelocity()
        vel_msg.is_sync = True
        vel_msg.is_tool_coord = False
        vel_msg.duration = 0.0 

        if len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            coords = box.xyxy[0].cpu().numpy().astype(int)
            c_bbox_x = (coords[0] + coords[2]) // 2
            c_bbox_y = (coords[1] + coords[3]) // 2

            err_x = c_img_x - c_bbox_x
            err_y = c_img_y - c_bbox_y
        
            # ACHTUNG: Mapping Bild-Y auf Roboter-X und Bild-X auf Roboter-Y
            vx = float(err_y * self.kp) if abs(err_y) > self.deadband else 0.0
            vy = float(err_x * self.kp) if abs(err_x) > self.deadband else 0.0

            vx = np.clip(vx, -self.max_v, self.max_v)
            vy = np.clip(vy, -self.max_v, self.max_v)

            vel_msg.speeds = [vx, vy, 0.0, 0.0, 0.0, 0.0]
            self.vel_publisher.publish(vel_msg)
        else:
            vel_msg.speeds = [0.0] * 6
            self.vel_publisher.publish(vel_msg)

        cv2.circle(annotated_frame, (c_img_x, c_img_y), 10, (0, 0, 255), 2)
        cv2.imshow("Calibrate Tray", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Calibrate_Tray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()