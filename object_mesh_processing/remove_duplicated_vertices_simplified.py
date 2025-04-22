import glob
import os
from multiprocessing import Pool

from tqdm import tqdm
import trimesh


def remove_duplicated_vertices(obj_path):
    mesh = trimesh.load(obj_path)

    # Performing vertex merging and face update
    mesh.merge_vertices(merge_norm=True, merge_tex=True)
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()

    mesh.export(obj_path)


if __name__ == '__main__':
    obj_paths = glob.glob('../../HILO_Dataset/HILO_Objects/simplified/*')
    with Pool(processes=os.cpu_count()) as pool:
        for _ in tqdm(pool.imap_unordered(remove_duplicated_vertices, obj_paths), total=len(obj_paths)):
            pass
