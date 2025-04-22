import glob
import os
import sys

import numpy as np
import cv2
import open3d as o3d
from tqdm import tqdm
from scipy.interpolate import LinearNDInterpolator
from scipy.spatial import cKDTree

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(os.path.join(ROOT_DIR))
from dataset_utils import CameraParameters, generate_cloud_points, generate_o3d_point_cloud


def construct_scene(color_path, depth_path, params):
    # Load color image
    color_img = cv2.imread(color_path, cv2.IMREAD_COLOR).astype(np.float32)
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
    h_c, w_c, _ = color_img.shape

    # Color rectification
    color_img_undistorted = cv2.undistort(color_img, params.color_K, params.distort_para_color)

    # Load depth image
    depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32)
    h_d, w_d = depth_img.shape

    # Prepare pixel coordinates
    y_indices, x_indices = np.meshgrid(np.arange(h_d), np.arange(w_d), indexing='ij')
    x_flat = x_indices.flatten().astype(np.float32)
    y_flat = y_indices.flatten().astype(np.float32)

    # Stack and reshape pixel coordinates
    pixel_coords = np.stack((x_flat, y_flat), axis=-1).reshape(-1, 1, 2)

    # Undistort pixel coordinates
    undistorted_coords = cv2.undistortPoints(pixel_coords, params.depth_K, params.distort_para_depth)
    undistorted_coords = undistorted_coords.reshape(-1, 2).T  # Shape: 2 x N

    # Compute 3D points in the depth camera coordinate frame
    z = depth_img.flatten()
    x_depth = undistorted_coords[0, :] * z
    y_depth = undistorted_coords[1, :] * z
    pts_depth = np.vstack((x_depth, y_depth, z))  # Shape: 3 x N

    # Transform to color camera coordinate frame
    pts_color = params.color_R.T @ pts_depth + params.color_t  # Shape: 3 x N

    # Handle points behind the camera
    valid_indices = (pts_color[2, :] > 0) & (pts_depth[2, :] > 0)  # Keep points where z > 0
    pts_color = pts_color[:, valid_indices]
    pts_depth = pts_depth[:, valid_indices]
    z = z[valid_indices]

    # Project onto color image plane
    color_coor = np.zeros((pts_color.shape[1], 2), dtype=np.float32)
    color_coor[:, 0] = ((pts_color[0, :] * params.color_K[0, 0]) / pts_color[2, :] + params.color_K[0, 2]).T
    color_coor[:, 1] = ((pts_color[1, :] * params.color_K[1, 1]) / pts_color[2, :] + params.color_K[1, 2]).T

    # Check for valid pixel coordinates within the color image bounds
    color_x = color_coor[:, 0].astype(int)
    color_y = color_coor[:, 1].astype(int)
    valid_mask = (
            (color_x >= 0) & (color_x < w_c) &
            (color_y >= 0) & (color_y < h_c)
    )
    valid_color_x = color_x[valid_mask]
    valid_color_y = color_y[valid_mask]
    valid_pts_depth = pts_depth[:, valid_mask]
    valid_z = z[valid_mask]

    # Get color values for valid points
    color_values = color_img_undistorted[valid_color_y, valid_color_x]

    # Create dense interpolation grid
    # Create grid of target image coordinates
    v_grid, u_grid = np.meshgrid(np.arange(h_d), np.arange(w_d), indexing='ij')
    grid_coords = np.stack((u_grid.flatten(), v_grid.flatten()), axis=-1).astype(np.float32)

    # Project valid 3D points back to image plane
    u_undistorted = (valid_pts_depth[0, :] * params.depth_K[0, 0]) / valid_pts_depth[2, :] + params.depth_K[0, 2]
    v_undistorted = (valid_pts_depth[1, :] * params.depth_K[1, 1]) / valid_pts_depth[2, :] + params.depth_K[1, 2]

    # Stack coordinates for interpolation
    valid_points = np.column_stack((u_undistorted, v_undistorted))

    # Perform interpolation
    tree = cKDTree(valid_points)
    distances, _ = tree.query(grid_coords)

    # Use LinearNDInterpolator for smooth interpolation
    interpolator_depth = LinearNDInterpolator(valid_points, valid_z, fill_value=0)
    interpolator_color = LinearNDInterpolator(valid_points, color_values, fill_value=0)

    # Interpolate values
    depth_interpolated = interpolator_depth(grid_coords)
    depth_interpolated[distances > 1.0] = 0
    color_interpolated = interpolator_color(grid_coords)

    # Clean up any invalid values
    depth_interpolated = np.nan_to_num(depth_interpolated, copy=False).astype(np.float32)
    color_interpolated = np.nan_to_num(color_interpolated, copy=False).astype(np.float32)
    depth_interpolated[depth_interpolated > 2000.0] = 0

    # Reconstruct point cloud
    pts_reconstructed = generate_cloud_points(depth_interpolated, h_d, w_d, params)
    pcd_reconstructed = generate_o3d_point_cloud(color_interpolated, pts_reconstructed)
    pcd_reconstructed, ind = pcd_reconstructed.remove_statistical_outlier(nb_neighbors=20, std_ratio=4.0)
    outlier_mask = np.ones(h_d * w_d, dtype=bool)
    ind_array = np.asarray(ind)
    outlier_mask[ind_array] = False
    depth_interpolated[outlier_mask] = 0

    # Reshape to image dimensions
    depth_img_reconstructed = depth_interpolated.reshape(h_d, w_d)
    color_img_reconstructed = color_interpolated.reshape(h_d, w_d, 3)

    # Post-process RGBD
    color_img_reconstructed = cv2.cvtColor(color_img_reconstructed.astype(np.uint8), cv2.COLOR_RGB2BGR)
    depth_img_reconstructed = depth_img_reconstructed.astype(np.uint16)

    return color_img_reconstructed, depth_img_reconstructed, pcd_reconstructed


def generate_scene_data(color_path):
    dir_path = os.path.dirname(color_path)
    depth_path = color_path.replace('rgb_raw', 'depth_raw')

    if 'arc0' in color_path or 'arc1' in color_path or 'arc2' in color_path or 'arc3' in color_path:
        params = params0123
    else:
        params = params4567

    rgb_undistorted, depth_undistorted, pcd_reconstructed = construct_scene(color_path,
                                                                            depth_path,
                                                                            params)
    color_undistorted_dir = dir_path.replace('rgb_raw', 'rgb_undistorted')
    if not os.path.exists(color_undistorted_dir):
        os.makedirs(color_undistorted_dir)
    color_undistorted_filename = os.path.basename(color_path)
    cv2.imwrite(os.path.join(color_undistorted_dir, color_undistorted_filename), rgb_undistorted)

    depth_undistorted_dir = dir_path.replace('rgb_raw', 'depth_undistorted')
    if not os.path.exists(depth_undistorted_dir):
        os.makedirs(depth_undistorted_dir)
    depth_undistorted_filename = os.path.basename(depth_path)
    cv2.imwrite(os.path.join(depth_undistorted_dir, depth_undistorted_filename), depth_undistorted)

    pcd_reconstructed_dir = dir_path.replace('rgb_raw', 'point_clouds_reconstructed')
    if not os.path.exists(pcd_reconstructed_dir):
        os.makedirs(pcd_reconstructed_dir)
    pcd_filename = color_undistorted_filename.replace('.png', '.ply')
    o3d.io.write_point_cloud(os.path.join(pcd_reconstructed_dir, pcd_filename), pcd_reconstructed)


if __name__ == '__main__':
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
        if os.path.isdir(scene_path):
            for rgb_path in glob.glob(os.path.join(scene_path, 'rgb_raw', '*')):
                generate_scene_data(os.path.join(rgb_path))
