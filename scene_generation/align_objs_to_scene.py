import os
import copy
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


def key_callback_q(vis):
    """Translate one step along X-"""
    global x
    displacement = translation_step
    x -= displacement
    obj_mesh.translate((-displacement, 0, 0), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_w(vis):
    """Translate one step along X+"""
    global x
    displacement = translation_step
    x += displacement
    obj_mesh.translate((displacement, 0, 0), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_e(vis):
    """Translate ten steps along X-"""
    global x
    displacement = translation_step * 10
    x -= displacement
    obj_mesh.translate((-displacement, 0, 0), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_r(vis):
    """Translate ten steps along X+"""
    global x
    displacement = translation_step * 10
    x += displacement
    obj_mesh.translate((displacement, 0, 0), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_a(vis):
    """Translate one step along Y-"""
    global y
    displacement = translation_step
    y -= displacement
    obj_mesh.translate((0, -displacement, 0), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_s(vis):
    """Translate one step along Y+"""
    global y
    displacement = translation_step
    y += displacement
    obj_mesh.translate((0, displacement, 0), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_d(vis):
    """Translate ten steps along Y-"""
    global y
    displacement = translation_step * 10
    y -= displacement
    obj_mesh.translate((0, -displacement, 0), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_f(vis):
    """Translate ten steps along Y+"""
    global y
    displacement = translation_step * 10
    y += displacement
    obj_mesh.translate((0, displacement, 0), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_z(vis):
    """Translate one step along Z-"""
    global z
    displacement = translation_step
    z -= displacement
    obj_mesh.translate((0, 0, -displacement), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_x(vis):
    """Translate one step along Z+"""
    global z
    displacement = translation_step
    z += displacement
    obj_mesh.translate((0, 0, displacement), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_c(vis):
    """Translate ten steps along Z-"""
    global z
    displacement = translation_step * 10
    z -= displacement
    obj_mesh.translate((0, 0, -displacement), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_v(vis):
    """Translate ten steps along Z+"""
    global z
    displacement = translation_step * 10
    z += displacement
    obj_mesh.translate((0, 0, displacement), relative=True)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()


def key_callback_o(vis):
    """Rotate one step around X-"""
    global x_rotation
    x_rotation -= rotation_step
    r_x = obj_mesh.get_rotation_matrix_from_xyz([np.radians(-rotation_step), 0, 0])
    obj_mesh.rotate(r_x)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()
    record_operation('rotate', axis='x', value=-rotation_step)


def key_callback_p(vis):
    """Rotate one step around X+"""
    global x_rotation
    x_rotation += rotation_step
    r_x = obj_mesh.get_rotation_matrix_from_xyz([np.radians(rotation_step), 0, 0])
    obj_mesh.rotate(r_x)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()
    record_operation('rotate', axis='x', value=rotation_step)


def key_callback_k(vis):
    """Rotate one step around Y-"""
    global y_rotation
    y_rotation -= rotation_step
    r_y = obj_mesh.get_rotation_matrix_from_xyz([0, np.radians(-rotation_step), 0])
    obj_mesh.rotate(r_y)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()
    record_operation('rotate', axis='y', value=-rotation_step)


def key_callback_l(vis):
    """Rotate one step around Y+"""
    global y_rotation
    y_rotation += rotation_step
    r_y = obj_mesh.get_rotation_matrix_from_xyz([0, np.radians(rotation_step), 0])
    obj_mesh.rotate(r_y)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()
    record_operation('rotate', axis='y', value=rotation_step)


def key_callback_n(vis):
    """Rotate one step around Z-"""
    global z_rotation
    z_rotation -= rotation_step
    r_z = obj_mesh.get_rotation_matrix_from_xyz([0, 0, np.radians(-rotation_step)])
    obj_mesh.rotate(r_z)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()
    record_operation('rotate', axis='z', value=-rotation_step)


def key_callback_m(vis):
    """Rotate one step around Z+"""
    global z_rotation
    z_rotation += rotation_step
    r_z = obj_mesh.get_rotation_matrix_from_xyz([0, 0, np.radians(rotation_step)])
    obj_mesh.rotate(r_z)
    vis.update_geometry(obj_mesh)
    vis.poll_events()
    vis.update_renderer()
    record_operation('rotate', axis='z', value=rotation_step)


def record_operation(operation, axis=None, value=None):
    """Record each operation (translation or rotation)"""
    operation_list.append({'operation': operation, 'axis': axis, 'value': value})


def compute_final_transformation(operations):
    """Compute the final transformation matrix based on recorded operations"""
    R_final = np.eye(3)

    for op in operations:
        angle = np.radians(op['value'])
        R = np.eye(3)

        if op['axis'] == 'x':
            R = o3d.geometry.get_rotation_matrix_from_xyz([angle, 0, 0])
        elif op['axis'] == 'y':
            R = o3d.geometry.get_rotation_matrix_from_xyz([0, angle, 0])
        elif op['axis'] == 'z':
            R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, angle])

        R_final = R @ R_final  # Apply rotation

    return R_final


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Align objects to the selected capture point.')
    parser.add_argument('--scene_name', type=str, default='scene_00_s0_c1',
                        help='Name of the scene to be aligned.')
    parser.add_argument('--arc_id', type=int, default=0,
                        help='Arc ID of the camera.')
    parser.add_argument('--cam_id', type=int, default=7,
                        help='Camera ID.')
    parser.add_argument('--translation_step', type=float, default=1.0,
                        help='Step size for translation.')
    parser.add_argument('--rotation_step', type=float, default=1.0,
                        help='Step size for rotation.')
    args = parser.parse_args()

    dataset_root = '../../HILO_Dataset'
    scenes_root = os.path.join(dataset_root, 'HILO_Scenes')
    scene_path = os.path.join(scenes_root, args.scene_name)
    print('scene_name', args.scene_name)

    output_file = os.path.join(scene_path, 'object_pose_wrt_arc0_image7.json')

    # Check if the output file already exists
    if os.path.exists(output_file):
        print('Loading existing object transformation from', output_file)
        trans_exists = True
    else:
        trans_exists = False

    # The flag to determine if the loaded transformation has been changed
    changed_flag = False

    # Load the transformation for arc0_image7 with respect to the table
    with open(os.path.join(scene_path, 'arc0_image7_wrt_table.json'), 'r') as f:
        arc0_7_wrt_table = np.array(json.load(f))

    # Load the camera pose for the current capture point
    with open(os.path.join(scene_path, 'camera_poses.json'), 'r') as f:
        camera_poses = json.load(f)
    current_cam_wrt_arc0_7 = np.asarray(camera_poses[f'arc{args.arc_id}_image{args.cam_id}'])

    # Reconstruct and transform the point cloud
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

    # Make a copy of the scene cloud in the table frame for object alignment.
    scene_cloud_wrt_table = copy.deepcopy(scene_cloud)
    scene_cloud_wrt_table.transform(arc0_7_wrt_table)
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])
    objs = []

    # Load the object ID list
    with open(os.path.join(scene_path, 'object_id_list.json'), 'r') as f:
        obj_ids = json.load(f)

    # Initialize the annotations list
    annotations = []
    for obj_id in obj_ids:
        obj_info = {'Model': obj_id, 'trans': np.eye(4)}
        annotations.append(obj_info)

    # Load and show the current object transformation if the annotation exists
    if trans_exists:
        with open(output_file, 'r') as f:
            obj_info_all = json.load(f)
        assert len(obj_info_all) == 10

        objs = []
        for i in range(len(obj_info_all)):
            obj_info_loaded = obj_info_all[i]
            obj_name_loaded = obj_info_loaded['Model']
            obj_trans = np.asarray(obj_info_loaded['trans'])

            obj_info = annotations[i]
            obj_name = obj_info['Model']
            assert obj_name_loaded == obj_name, f'Object names do not match: {obj_name_loaded} != {obj_name}'
            annotations[i]['trans'] = obj_trans

            obj_mesh = o3d.io.read_triangle_mesh(os.path.join(dataset_root,
                                                              'HILO_Objects',
                                                              'simplified',
                                                              f'{obj_name}_simplified.obj'))
            obj_mesh.paint_uniform_color([1, 0, 0])
            obj_mesh.transform(obj_trans)
            objs.append(obj_mesh)

        o3d.visualization.draw_geometries([
            scene_cloud,
            *objs,
            coordinate_frame
        ])

    # Annotations
    for i in range(len(annotations)):
        annotation = annotations[i]
        obj_name = annotation['Model']
        obj_trans_wrt_cam = annotation['trans']
        obj_trans = arc0_7_wrt_table @ obj_trans_wrt_cam
        obj_mesh = o3d.io.read_triangle_mesh(os.path.join(dataset_root,
                                                          'HILO_Objects',
                                                          'simplified',
                                                          f'{obj_name}_simplified.obj'))
        obj_mesh.paint_uniform_color([1, 0, 0])

        obj_mesh_copy = copy.deepcopy(obj_mesh)
        obj_mesh_copy.paint_uniform_color([0, 0, 1])

        obj_mesh.transform(obj_trans)
        center = obj_mesh.get_center()

        x, y, z = 0, 0, 0
        x_rotation, y_rotation, z_rotation = 0, 0, 0
        T_ini = obj_trans
        operation_list = []
        translation_step = args.translation_step
        rotation_step = args.rotation_step

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window('Interactive Point Cloud Viewer', width=1200, height=900)

        vis.register_key_callback(ord('Q'), key_callback_q)  # Translate one step along X-
        vis.register_key_callback(ord('W'), key_callback_w)  # Translate one step along X+
        vis.register_key_callback(ord('E'), key_callback_e)  # Translate ten steps along X-
        vis.register_key_callback(ord('R'), key_callback_r)  # Translate ten steps along X+

        vis.register_key_callback(ord('A'), key_callback_a)  # Translate one step along Y-
        vis.register_key_callback(ord('S'), key_callback_s)  # Translate one step along Y+
        vis.register_key_callback(ord('D'), key_callback_d)  # Translate ten steps along Y-
        vis.register_key_callback(ord('F'), key_callback_f)  # Translate ten steps along Y+

        vis.register_key_callback(ord('Z'), key_callback_z)  # Translate one step along Z-
        vis.register_key_callback(ord('X'), key_callback_x)  # Translate one step along Z+
        vis.register_key_callback(ord('C'), key_callback_c)  # Translate ten steps along Z-
        vis.register_key_callback(ord('V'), key_callback_v)  # Translate ten steps along Z+

        vis.register_key_callback(ord('O'), key_callback_o)  # Rotate one step around X-
        vis.register_key_callback(ord('P'), key_callback_p)  # Rotate one step around X+
        vis.register_key_callback(ord('K'), key_callback_k)  # Rotate one step around Y-
        vis.register_key_callback(ord('L'), key_callback_l)  # Rotate one step around Y+
        vis.register_key_callback(ord('N'), key_callback_n)  # Rotate one step around Z-
        vis.register_key_callback(ord('M'), key_callback_m)  # Rotate one step around Z+

        vis.add_geometry(scene_cloud_wrt_table)
        vis.add_geometry(obj_mesh)
        vis.add_geometry(coordinate_frame)

        vis.run()
        vis.destroy_window()

        # Get the final transformation
        if not (x == 0 and y == 0 and z == 0 and x_rotation == 0 and y_rotation == 0 and z_rotation == 0):
            print(f'{obj_name}: transformation changed.')
            changed_flag = True

            T0 = np.eye(4)
            T0[:3, :3] = compute_final_transformation(operation_list)
            T0[:3, 3] = [x, y, z]

            T_center_to_origin = np.eye(4)
            T_center_to_origin[:3, 3] = -center
            T_back_to_center = np.eye(4)
            T_back_to_center[:3, 3] = center
            T = T_back_to_center @ T0 @ T_center_to_origin
            T = T @ T_ini
            T = np.linalg.inv(arc0_7_wrt_table) @ T

            # Check the final alignment
            o3d.visualization.draw_geometries([
                scene_cloud,
                obj_mesh_copy.transform(T),
                coordinate_frame
            ])

            annotations[i]['trans'] = T.tolist()

    # Save the updated annotations
    if changed_flag:
        if trans_exists:
            ans = input('Overwrite the existing transformation? (y/n): ')
            if ans.lower() != 'y':
                print('Changes not saved.')

            else:
                print('Saving changes to', output_file)
                with open(output_file, 'w') as output_f:
                    json.dump(annotations, output_f, indent=2)
