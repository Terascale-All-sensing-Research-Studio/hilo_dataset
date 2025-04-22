import json
import numpy as np
import open3d as o3d


class CameraParameters:
    def __init__(self, camera_parameters_path):
        self.camera_parameters_path = camera_parameters_path
        with open(camera_parameters_path, 'r') as f:
            camera_parameters = json.load(f)

        self.depth_K = np.asarray(camera_parameters['depth_K'])
        self.color_K = np.asarray(camera_parameters['color_K'])
        self.color_R = np.asarray(camera_parameters['color_R'])
        self.color_t = np.asarray(camera_parameters['color_t'])
        self.distort_para_depth = np.asarray(camera_parameters['distort_para_depth'])
        self.distort_para_color = np.asarray(camera_parameters['distort_para_color'])

    @property
    def RT3(self):
        return np.hstack((self.color_R, self.color_t))

    @property
    def RT4(self):
        return np.vstack((self.RT3, np.array([0, 0, 0, 1])))


def generate_cloud_points(depth_array, h, w, paras):
    fx, fy = paras.depth_K[0, 0], paras.depth_K[1, 1]
    cx, cy = paras.depth_K[0, 2], paras.depth_K[1, 2]

    # Generate grid of (x, y) coordinates
    x_coords, y_coords = np.meshgrid(np.arange(w), np.arange(h), indexing='xy')

    # Flatten and normalize pixel coordinates to get 3D points
    z = depth_array  # Depth values
    x = (x_coords.flatten() - cx) * z / fx  # Convert pixel x to camera x
    y = (y_coords.flatten() - cy) * z / fy  # Convert pixel y to camera y

    # Stack x, y, z, r, g, b to form a point cloud array
    pts = np.stack((x, y, z), axis=-1)

    return pts


def generate_o3d_point_cloud(color_array, points_array):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_array)
    point_cloud.colors = o3d.utility.Vector3dVector(color_array / 255.0)

    return point_cloud
