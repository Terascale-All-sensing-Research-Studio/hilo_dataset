import os
import json
import sys
import argparse

import numpy as np
import open3d as o3d
import cv2

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)
sys.path.append(os.path.join(ROOT_DIR))
from dataset_utils import CameraParameters, generate_cloud_points, generate_o3d_point_cloud

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='View the selected scene with aligned objects.')
    parser.add_argument('--scene_name', type=str, default='scene_00_s0_c1',
                        help='Name of the scene to be aligned.')
    parser.add_argument('--arc_id', type=int, default=0,
                        help='Arc ID of the camera.')
    parser.add_argument('--cam_id', type=int, default=7,
                        help='Camera ID.')
    args = parser.parse_args()

    dataset_root = '../../HILO_Dataset'
    scenes_root = os.path.join(dataset_root, 'HILO_Scenes')
    scene_path = os.path.join(scenes_root, args.scene_name)
    print('scene_name', args.scene_name)

    # Load the transformation for arc0_image7 with respect to the table
    with open(os.path.join(scene_path, 'arc0_image7_wrt_table.json'), 'r') as f:
        arc0_7_wrt_table = np.array(json.load(f))

    # Load the camera pose for the current capture point
    with open(os.path.join(scene_path, 'camera_poses.json'), 'r') as f:
        camera_poses = json.load(f)
    current_cam_wrt_arc0_7 = np.asarray(camera_poses[f'arc{args.arc_id}_image{args.cam_id}'])

    color_path = os.path.join(scene_path, 'rgb_undistorted', f'arc{args.arc_id}_image{args.cam_id}.png')
    color_img = cv2.imread(color_path, cv2.IMREAD_COLOR).astype(np.float32)
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
    depth_path = os.path.join(scene_path, 'depth_undistorted', f'arc{args.arc_id}_image{args.cam_id}.png')
    depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32)
    h_d, w_d = depth_img.shape
    if args.arc_id in [0, 1, 2, 3]:
        camera_parameters_path = os.path.join(scenes_root, 'cam_params_arc0123.json')
    else:
        camera_parameters_path = os.path.join(scenes_root, 'cam_params_arc4567.json')
    params = CameraParameters(camera_parameters_path)
    cloud_points = generate_cloud_points(depth_img.flatten(), h_d, w_d, params)
    scene_cloud = generate_o3d_point_cloud(color_img.reshape(h_d * w_d, 3), cloud_points)
    scene_cloud = scene_cloud.transform(current_cam_wrt_arc0_7)

    annotation_path = os.path.join(scene_path, 'object_trans_wrt_arc0_image7.json')
    with open(annotation_path, 'r') as fj:
        annotations = json.load(fj)

    objs = []
    for annotation in annotations:
        obj_name = annotation['Model']
        obj_trans = np.asarray(annotation['trans'])
        obj_mesh = o3d.io.read_triangle_mesh(os.path.join(dataset_root,
                                                          'HILO_Objects',
                                                          'simplified',
                                                          f'{obj_name}_simplified.obj'))
        obj_mesh.paint_uniform_color([1, 0, 0])
        obj_mesh.transform(obj_trans)
        objs.append(obj_mesh)

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([
        scene_cloud,
        *objs,
        coordinate_frame
    ])
