""" Demo to show prediction results.
    Author: chenxi-wang
"""

import os
import sys
import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image

import torch
from graspnetAPI import GraspGroup, GraspNet

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from graspnet import GraspNet, pred_decode
from graspnet_dataset import GraspNetDataset
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', required=True, help='Model checkpoint path')
parser.add_argument('--num_point', type=int, default=20000, help='Point Number [default: 20000]')
parser.add_argument('--num_view', type=int, default=300, help='View Number [default: 300]')
parser.add_argument('--collision_thresh', type=float, default=0.01, help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--voxel_size', type=float, default=0.01, help='Voxel Size to process point clouds before collision detection [default: 0.01]')
cfgs = parser.parse_args()


def get_net():
    # Init the model
    net = GraspNet(input_feature_dim=0, num_view=cfgs.num_view, num_angle=12, num_depth=4,
            cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    #device=torch.device("cpu")
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> loaded checkpoint %s (epoch: %d)"%(cfgs.checkpoint_path, start_epoch))
    # set model to eval mode
    net.eval()
    return net

def get_and_process_data(data_dir):
    # load data
    color = np.array(Image.open(os.path.join(scene_dir, 'color.png')), dtype=np.float32) / 255.0
    depth = np.array(Image.open(os.path.join(scene_dir, 'depth.png')))
    workspace_mask = np.array(Image.open(os.path.join(scene_dir, 'full_workspace_mask.png')))
    #meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
    #intrinsic = meta['intrinsic_matrix']
    #factor_depth = meta['factor_depth']
    factor_depth = 1000

    # generate cloud
    #camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2], factor_depth)
    #Camera_Intrinsics of depth
    #camera = CameraInfo(1280.0 , 720.0, 641.1640625, 641.1640625, 642.5853271484375, 359.3363037109375, factor_depth)
    #Camera_intrinsics of Color
    camera = CameraInfo(1280.0, 720.0, 912.0184936523438, 911.9789428710938, 652.2272338867188, 377.44488525390625, factor_depth)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # get valid points
    mask = workspace_mask & (depth > 150) & (depth < 532)
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    print("workspace_mask unique values:", np.unique(workspace_mask))
    print("depth min/max:", depth.min(), depth.max())
    print("Valid points after mask:", len(cloud_masked))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    pcd.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

    # 1) Statistische Outlier-Entfernung
    # pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.5)
    # cloud_masked = np.asarray(pcd.points)
    # color_masked = np.asarray(pcd.colors)
    # radius outlier
    # pcd, ind = pcd.remove_radius_outlier(nb_points=200, radius=0.02)
    # cloud_masked = np.asarray(pcd.points)
    # color_masked = np.asarray(pcd.colors)

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point-len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud

def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg

def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg

def vis_grasps(gg, cloud):
    gg.nms()
    gg.sort_by_score()
    
     # Step 1: Transform Z-axis for better top-down visualization
    flip_z = np.array([
        [1, 0,  0, 0],
        [0, -1,  0, 0],
        [0, 0, -1, 0],
        [0, 0,  0, 1]
    ])
    cloud.transform(flip_z)
    top_k = 100
    grippers = gg[:top_k].to_open3d_geometry_list()
    for gripper in grippers:
        gripper.transform(flip_z)

    # Step 2: Visualize all grasps
    o3d.visualization.draw_geometries([cloud, *grippers], window_name="All Grasps")

    # Step 3: Visualize only the top grasp
    top_gripper = gg[0].to_open3d_geometry()
    top_gripper.transform(flip_z)
    o3d.visualization.draw_geometries([cloud, top_gripper], window_name="Top Grasp Only")

    
def save_grasps(gg, save_path='predicted_grasps.npy'):
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    gg.save_npy(save_path)
    print(f"Saved {len(gg)} grasps to '{save_path}'")

def demo(data_dir):
    net = get_net()
    end_points, cloud = get_and_process_data(data_dir)
    gg = get_grasps(net, end_points)
    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
        
    gg.sort_by_score()  # ✅ Sort before saving      
    save_grasps(gg, os.path.join(data_dir, 'predicted_grasps.npy')) 
    #vis_grasps(gg, cloud)

if __name__=='__main__':
    scene_dir = './scene_images'
    data_dir = './scene_data'
    demo(data_dir)
