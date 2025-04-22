import glob
import os
from multiprocessing import Pool

from tqdm import tqdm
import pymeshlab


def simplify_mesh(obj_path):
    ms = pymeshlab.MeshSet()

    # Load the mesh
    ms.load_new_mesh(obj_path)

    # Simplify the mesh to 20k faces
    ms.meshing_decimation_quadric_edge_collapse(
        targetfacenum=20000,
        preservenormal=True,
        preservetopology=False,  # Avoids topological changes like creating holes
        preserveboundary=True,  # Keeps boundaries intact
        qualitythr=0.3,  # Higher threshold keeps more detailed edges
        optimalplacement=True  # Ensures better placement of vertices
    )

    # Export the simplified mesh
    export_name = os.path.basename(obj_path).replace('_cleaned.obj', '_simplified.obj')
    export_path = os.path.join('../../HILO_Dataset/HILO_Objects/simplified', export_name)
    ms.save_current_mesh(export_path)

    ms.clear()


if __name__ == '__main__':
    obj_paths = glob.glob('../../HILO_Dataset/HILO_Objects/full/*')
    with Pool(processes=os.cpu_count()) as pool:
        for _ in tqdm(pool.imap_unordered(simplify_mesh, obj_paths), total=len(obj_paths)):
            pass
