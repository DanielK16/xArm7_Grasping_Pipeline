"""
Visualize saved grasps using the GraspGroup class.
"""

import os
import sys
import numpy as np
import open3d as o3d
import argparse
import scipy.io as scio
from PIL import Image
from graspnetAPI import GraspGroup
from screeninfo import get_monitors  # Added for screen size

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from data_utils import CameraInfo, create_point_cloud_from_depth_image

parser = argparse.ArgumentParser()
parser.add_argument('--data_dir', default='/home/rric/GRASP/graspnet-baseline/scene_images', help='Path to the data directory')
parser.add_argument('--grasp_path', default='/home/rric/GRASP/graspnet-baseline/scene_data/predicted_grasps.npy', help='Path to saved grasp file')
parser.add_argument('--mode', choices=['raw', 'orientation_only', 'mask_only', 'orientation_mask'], default='raw', help='Choose which filters to apply to grasps')
parser.add_argument('--view', choices=['all', 'top'], default='all', help='Choose to visuslise all grasps or only the top 1 grasp')

args = parser.parse_args()


def load_point_cloud(data_dir):
    # Load data
    color = np.array(Image.open(os.path.join(data_dir, 'color.png')), dtype=np.float32) / 255.0
    depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))
    workspace_mask = np.array(Image.open(os.path.join(data_dir, 'full_workspace_mask.png')))
    factor_depth = 1000.0

    # Use known camera intrinsics
    camera = CameraInfo(
        width=1280.0, height=720.0,
        fx=912.0184936523438, fy=911.9789428710938,
        cx=652.2272338867188, cy=377.44488525390625,
        scale=factor_depth
    )
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # Mask
    mask = workspace_mask & (depth > 0) & (depth < 700)
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    # Convert to Open3D format
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    pcd.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    return pcd


def visualize_geometries_window(window_name, geometries):
    # Utility to create sized window and visualize given geometries
    monitor = get_monitors()[0]
    screen_width = monitor.width
    screen_height = monitor.height

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=window_name,
        width=screen_width // 2,
        height=screen_height // 2,
        left=0,
        top=0
    )

    for geom in geometries:
        vis.add_geometry(geom)

    vis.run()
    vis.destroy_window()


def vis_grasps(gg, cloud, view):
    gg.nms()
    gg.sort_by_score()

    flip_z = np.array([
        [1, 0,  0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0,  0, 1]
    ])
    cloud.transform(flip_z)
    
    #view all grasps wrt mode
    if view == "all":
        grippers = gg[:200].to_open3d_geometry_list()
        for gripper in grippers:
            gripper.transform(flip_z)
        visualize_geometries_window("All Grasps", [cloud, *grippers])
        
    #view top 1 grasp wrt mode
    if view == "top":
        grippers = gg[:1].to_open3d_geometry_list()
        for gripper in grippers:
            gripper.transform(flip_z)
        visualize_geometries_window("Top Grasps", [cloud, *grippers])






def main():
    cloud = load_point_cloud(args.data_dir)
    grasp_file = args.grasp_path if os.path.isabs(args.grasp_path) else os.path.join(args.data_dir, args.grasp_path)
    if not os.path.exists(grasp_file):
        print(f"Error: grasp file not found at {grasp_file}")
        return

    all_grasps_array = np.load(grasp_file)  # shape (N, 21)
    # Filter grasps element is 1
    #filtered_grasps_array = all_grasps_array[(all_grasps_array[:, 17] == 1)]
    if args.mode == 'raw':
        filtered_grasps_array = all_grasps_array
        print(f"Loaded {len(filtered_grasps_array)} raw grasps.")

    elif args.mode == 'orientation_only':
        filtered_grasps_array = all_grasps_array[all_grasps_array[:, 17] == 1]
        print(f"Filtered {len(all_grasps_array)} -> {len(filtered_grasps_array)} orientation-valid grasps.")

    elif args.mode == 'mask_only':
        filtered_grasps_array = all_grasps_array[all_grasps_array[:, 20] == 1]
        print(f"Filtered {len(all_grasps_array)} -> {len(filtered_grasps_array)} mask-valid grasps.")
        
    elif args.mode == 'orientation_mask':
        filtered_grasps_array = all_grasps_array[(all_grasps_array[:, 17] == 1) & (all_grasps_array[:, 20] == 1) ]
        print(f"Filtered {len(all_grasps_array)} -> {len(filtered_grasps_array)} orientation & mask-valid grasps.")


    # Create GraspGroup from filtered grasps
    gg = GraspGroup(filtered_grasps_array)

    # Optional: scale grasp widths or any other modifications here
    for g in gg.grasp_group_array:
        g[1] = g[1] * 1.0  # example scaling if needed
        
        
    vis_grasps(gg, cloud, view=args.view)


if __name__ == '__main__':
    main()
