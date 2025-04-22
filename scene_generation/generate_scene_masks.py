import os
import json
import glob
import sys

import numpy as np
import cv2
from tqdm import tqdm

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(os.path.join(ROOT_DIR))
from dataset_utils import CameraParameters, generate_cloud_points, generate_o3d_point_cloud


def filter_scene(color_path, depth_path, params):
    """Select all points within a cylinder in the scene."""
    capture_name = os.path.basename(color_path).replace('.png', '')

    # Load color and depth images
    color_img = cv2.imread(color_path, cv2.IMREAD_COLOR).astype(np.float32)
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

    depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32)
    h, w, _ = color_img.shape

    # Flatten the depth image
    depth_array = depth_img.flatten()

    # Filter the point cloud based on the scene mask
    points_array = generate_cloud_points(depth_array, h, w, params)
    points_homogeneous = np.hstack((points_array, np.ones((points_array.shape[0], 1))))
    T = arc0_7_wrt_table @ camera_poses[capture_name]
    transformed_points_homogeneous = (T @ points_homogeneous.T).T
    transformed_points = transformed_points_homogeneous[:, :3]
    distance_squared = transformed_points[:, 0] ** 2 + transformed_points[:, 1] ** 2
    inside = (
            (distance_squared <= cylinder_radius ** 2) &
            (transformed_points[:, 2] >= cylinder_bottom_z) &
            (transformed_points[:, 2] <= cylinder_top_z)
    )

    # Create a binary mask based on the containment results
    mask = np.zeros((h, w), dtype=np.uint8)
    inside_indices = np.where(inside)[0]

    # Map point cloud indices to depth image pixels
    pixel_y = inside_indices // w
    pixel_x = inside_indices % w
    mask[pixel_y, pixel_x] = 1
    mask = (mask * 255).astype(np.uint8)

    return mask


def generate_scene_masks(color_path):
    dir_path = os.path.dirname(color_path)
    depth_path = color_path.replace('rgb_undistorted', 'depth_undistorted')

    if 'arc0' in color_path or 'arc1' in color_path or 'arc2' in color_path or 'arc3' in color_path:
        params = params0123
    else:
        params = params4567

    mask = filter_scene(color_path, depth_path, params)
    mask_dir = dir_path.replace('rgb_undistorted', 'mask_undistorted')
    if not os.path.exists(mask_dir):
        os.makedirs(mask_dir)
    mask_filename = os.path.basename(color_path)
    cv2.imwrite(os.path.join(mask_dir, mask_filename), mask)


if __name__ == '__main__':
    cylinder_radius = 890 / 2  # Diameter divided by 2
    cylinder_bottom_z = -50
    cylinder_top_z = 400

    dataset_root = '../../HILO_Dataset'
    scenes_root = os.path.join(dataset_root, 'HILO_Scenes')

    # Load camera parameters
    params0123 = CameraParameters(os.path.join(scenes_root, 'cam_params_arc0123.json'))
    params4567 = CameraParameters(os.path.join(scenes_root, 'cam_params_arc4567.json'))

    # Gather all scene paths
    all_scene_paths = []
    for scene_path in glob.glob(os.path.join(scenes_root, '*')):
        if os.path.isdir(scene_path):
            all_scene_paths.append(scene_path)

    # Process each scene
    for scene_path in tqdm(all_scene_paths):
        with open(os.path.join(scene_path, 'arc0_image7_wrt_table.json'), 'r') as f:
            arc0_7_wrt_table = np.array(json.load(f))

        with open(os.path.join(scene_path, 'camera_poses.json'), 'r') as f:
            camera_poses = json.load(f)

        for rgb_path in glob.glob(os.path.join(scene_path, 'rgb_undistorted', '*')):
            generate_scene_masks(os.path.join(rgb_path))
