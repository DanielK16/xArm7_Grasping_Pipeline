#!/usr/bin/env python3
"""
Creates a static Planning Scene for MoveIt2 with collision objects
Publishes to topic /planning_scene
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# Import PlanningScene and ObjectColor
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor, AllowedCollisionEntry
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

import trimesh
from pymoveit2 import MoveIt2

class SimpleSceneSpawner(Node):
    def __init__(self):
        super().__init__('simple_scene_spawner')
        
        # Create Publisher 
        self.pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Publish to topic every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_scene)
        
        self.get_logger().info("[Scene_Spawner] Active. Publishing scene objects every 2 seconds...")

    def create_box_object(self, name, frame_id, dimensions, position, orientation=[0.0, 0.0, 0.0, 1.0]):
        """
        Helper function to create a collision object (Box)
        """
        co = CollisionObject()
        co.header.frame_id = frame_id
        co.id = name
        
        # Shape
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX     #other Primitives like (.SPHERE .CONE .CYLINDER can be added!)
        primitive.dimensions = dimensions 
        
        # Pose
        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = float(orientation[0])
        pose.orientation.y = float(orientation[1])
        pose.orientation.z = float(orientation[2])
        pose.orientation.w = float(orientation[3])
        
        # Assemble
        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        
        return co

    def create_color(self, id, r, g, b, a=1.0):
        """
        r, g, b, a are floats between 0.0 and 1.0
        a = alpha (transparency of object 1=opaque, 0=transparent)
        example colours (rgb):
        dark brown = (0.6,0.4,0.2)
        dark grey = (0.3, 0.3, 0.3)
        light grey = (0.7, 0.7, 0.7)
        red = (0.8, 0.1, 0.1)
        blue = (0.1, 0.1, 0.8)
        green = (0.1, 0.8, 0.1)
        """
        color = ObjectColor()
        color.id = id
        color.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
        return color

    def publish_scene(self):

        scene = PlanningScene()
        scene.is_diff = True 
        
        # --- Object 1: Table Tray ---
        table_tray = self.create_box_object(
            name="table_tray",
            frame_id="link_base",
            dimensions=[0.8, 0.75, 0.025],
            position=[0.54, 0.0, -0.085]
        )
        scene.world.collision_objects.append(table_tray)
        scene.object_colors.append(self.create_color("table_tray", 0.6, 0.4, 0.2)) # Brown

        # --- Object 2: Side Tray ---
        tray = self.create_box_object(
            name="tray",
            frame_id="link_base",
            dimensions=[0.4, 0.6, 0.005],
            position=[0.4, 0.0, 0.06]
        )
        scene.world.collision_objects.append(tray)
        scene.object_colors.append(self.create_color("tray", 0.8, 0.1, 0.1)) # Red

        # --- Object 3: Robot Plate ---
        robot_plate = self.create_box_object(
            name="robot_plate",
            frame_id="link_base",
            dimensions=[0.65, 0.9, 0.44],
            position=[-0.225, -0.3, -0.24]
        )
        scene.world.collision_objects.append(robot_plate)
        scene.object_colors.append(self.create_color("robot_plate", 0.7, 0.7, 0.7)) # Light Grey

        # --- Object 4: Floor ---
        floor = self.create_box_object(
            name="floor",
            frame_id="link_base",
            dimensions=[3.0, 3.0, 0.03],
            position=[0.0, 0.0, -0.45]
        )
        scene.world.collision_objects.append(floor)
        scene.object_colors.append(self.create_color("floor", 0.4, 0.2, 0.1)) #Brown

        # --- Object 5: Camera Pole ---
        camera_pole = self.create_box_object(
            name="camera_pole",
            frame_id="link_base",
            dimensions=[0.09, 0.05, 1.5],
            position=[-0.5, 0.0, 0.0]
        )
        scene.world.collision_objects.append(camera_pole)
        scene.object_colors.append(self.create_color("camera_pole", 0.7, 0.7, 0.7)) # Light Grey

        # --- Object 6: Showcase Plate for Meshes ---
        showcase_plate = self.create_box_object(
            name="showcase_plate",
            frame_id="link_base",
            dimensions=[0.65, 0.9, 0.44],
            position=[-0.225, 1.25, -0.24]
        )
        # add a object without collision to solely act as visual
        scene.world.collision_objects.append(showcase_plate)
        scene.allowed_collision_matrix.default_entry_names.append("showcase_plate")
        scene.allowed_collision_matrix.default_entry_values.append(True)
        scene.object_colors.append(self.create_color("showcase_plate", 0.7, 0.7, 0.7, a=0.5)) # Light Grey
        
        # Publish the full scene update 
        self.pub.publish(scene)
        #self.get_logger().info(f"Published scene objects: table_tray, tray, robot_plate")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSceneSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()