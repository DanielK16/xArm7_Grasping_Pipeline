#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import os
import trimesh 

# ROS 2 Messages
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import PlanningScene, ObjectColor, CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from std_msgs.msg import ColorRGBA

class SceneSpawner(Node):
    """
    Publishes the static Planning Scene.
    """

    def __init__(self):
        super().__init__('scene_spawner_node')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.get_logger().info("SceneSpawner initialized. Waiting for MoveIt connection...")
        time.sleep(2.0) 

    # =========================================================================
    # HELPER FUNCTIONS
    # =========================================================================

    def create_primitive(self, type_id, dimensions):
        p = SolidPrimitive()
        p.type = type_id
        p.dimensions = [float(x) for x in dimensions]
        return p

    def create_pose(self, pos, orientation=[0.0, 0.0, 0.0, 1.0]):
        p = Pose()
        p.position.x = float(pos[0])
        p.position.y = float(pos[1])
        p.position.z = float(pos[2])
        p.orientation.x = float(orientation[0])
        p.orientation.y = float(orientation[1])
        p.orientation.z = float(orientation[2])
        p.orientation.w = float(orientation[3])
        return p

    def create_color(self, obj_id, r, g, b, a=1.0):
        c = ObjectColor()
        c.id = obj_id
        c.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
        return c

    def create_collision_object(self, obj_id, frame_id, primitive, pose):
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = frame_id
        co.operation = CollisionObject.ADD
        
        if isinstance(primitive, SolidPrimitive):
            co.primitives.append(primitive)
            co.primitive_poses.append(pose)
        elif isinstance(primitive, Mesh):
            co.meshes.append(primitive)
            co.mesh_poses.append(pose)
        return co

    def load_mesh(self, filepath, scale=(1.0, 1.0, 1.0)):
        if not os.path.exists(filepath):
            self.get_logger().error(f"Mesh file not found: {filepath}")
            return None
        try:
            mesh_data = trimesh.load(filepath)
            msg = Mesh()
            for v in mesh_data.vertices:
                msg.vertices.append(Point(x=v[0]*scale[0], y=v[1]*scale[1], z=v[2]*scale[2]))
            for f in mesh_data.faces:
                tri = MeshTriangle()
                tri.vertex_indices = [int(f[0]), int(f[1]), int(f[2])]
                msg.triangles.append(tri)
            return msg
        except Exception as e:
            self.get_logger().error(f"Failed to load mesh: {e}")
            return None
    
    def add_box(self, scene, name, size, pos, color):
        """Adds a box to the scene object """
        prim = self.create_primitive(SolidPrimitive.BOX, size)
        pose = self.create_pose(pos)
        co = self.create_collision_object(name, "link_base", prim, pose)
        
        # Append to scene
        scene.world.collision_objects.append(co)
        scene.object_colors.append(self.create_color(name, *color))
        print(f"[INFO] Added Box: {name}")

    # =========================================================================
    # MAIN SCENE LOGIC
    # =========================================================================

    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True 

        # ---------------------------------------------------------------------
        # ROBOT WORKSPACE 
        # ---------------------------------------------------------------------
        print("[INFO] Building Robot Workspace...")

        # 1. Floor (Dark Grey)
        self.add_box(scene, "floor", 
                     size=[3.0, 3.0, 0.03], 
                     pos=[0.0, 0.0, -0.45], 
                     color=[0.2, 0.2, 0.2])

        # 2. Main Table (Wood)
        self.add_box(scene, "table_tray", 
                     size=[0.8, 0.75, 0.025], 
                     pos=[0.54, 0.0, -0.085], 
                     color=[0.6, 0.4, 0.2])

        # 3. Robot Base Plate (Aluminium)
        self.add_box(scene, "robot_plate", 
                     size=[0.65, 0.9, 0.44], 
                     pos=[-0.225, -0.3, -0.24], 
                     color=[0.7, 0.7, 0.7])

        # 4. Side Tray (Red)
        self.add_box(scene, "side_tray", 
                     size=[0.4, 0.6, 0.005], 
                     pos=[0.4, 0.0, 0.06], 
                     color=[0.8, 0.1, 0.1])

        # 5. Camera Pole (Grey)
        self.add_box(scene, "camera_pole", 
                     size=[0.09, 0.05, 1.5], 
                     pos=[-0.5, 0.0, 0.0], 
                     color=[0.8, 0.8, 0.8])
        
        # 6. The Showcase Table
        self.add_box(scene, "showcase_table", 
                     size=[1.2, 1.0, 0.4], 
                     pos=[0.0, 1.2, -0.25], 
                     color=[0.9, 0.9, 0.9])

        # 7. Example SPHERE
        r_s = 0.05
        co_s = self.create_collision_object(
            "ex_sphere", "link_base", 
            self.create_primitive(SolidPrimitive.SPHERE, [r_s]), 
            self.create_pose([-0.15, 1.2, -0.05 + r_s])
        )
        scene.world.collision_objects.append(co_s)
        scene.object_colors.append(self.create_color("ex_sphere", 0.0, 1.0, 0.0))

        # 8. Example CYLINDER
        h_c, r_c = 0.15, 0.03
        co_c = self.create_collision_object(
            "ex_cylinder", "link_base", 
            self.create_primitive(SolidPrimitive.CYLINDER, [h_c, r_c]), 
            self.create_pose([0.0, 1.2, -0.05 + h_c/2]) 
        )
        scene.world.collision_objects.append(co_c)
        scene.object_colors.append(self.create_color("ex_cylinder", 0.0, 0.0, 1.0))

        # 9. Example CONE
        h_cone, r_cone = 0.15, 0.04
        co_cone = self.create_collision_object(
            "ex_cone", "link_base", 
            self.create_primitive(SolidPrimitive.CONE, [h_cone, r_cone]), 
            self.create_pose([0.15, 1.2, -0.05 + h_cone/2]) 
        )
        scene.world.collision_objects.append(co_cone)
        scene.object_colors.append(self.create_color("ex_cone", 1.0, 1.0, 0.0))

        # 10. Example BOX 
        box_dims = [0.1, 0.1, 0.15] 
        co_box = self.create_collision_object(
            "ex_box", "link_base", 
            self.create_primitive(SolidPrimitive.BOX, box_dims), 
            self.create_pose([-0.3, 1.2, -0.05 + (box_dims[2]/2)]) 
        )
        scene.world.collision_objects.append(co_box)
        scene.object_colors.append(self.create_color("ex_box", 1.0, 0.0, 1.0))

        # 11. Example Mesh (ROTATED 180 deg around Y)
        mesh_path = "/home/rric/AI_Planner/GraspingPlanner/PC_2_MESH/output_data/bowl_green.stl"
        mesh_prim = self.load_mesh(mesh_path)

        if mesh_prim:
            q_rotate_180_y = [0.0, 1.0, 0.0, 0.0]
            mesh_pose = self.create_pose(
                [0.35, 1.2, 0.05], 
                orientation=q_rotate_180_y 
            )
            co_mesh = self.create_collision_object("user_mesh_demo", "link_base", mesh_prim, mesh_pose)
            scene.world.collision_objects.append(co_mesh)
            scene.object_colors.append(self.create_color("user_mesh_demo", 1.0, 0.0, 1.0))

        # ---------------------------------------------------------------------
        # PUBLISH
        # ---------------------------------------------------------------------
        print(f"[INFO] Publishing {len(scene.world.collision_objects)} objects to MoveIt...")
        self.scene_pub.publish(scene)


def main(args=None):
    rclpy.init(args=args)
    node = SceneSpawner()
    node.publish_scene()
    
    try:
        rclpy.spin_once(node, timeout_sec=2.0)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()