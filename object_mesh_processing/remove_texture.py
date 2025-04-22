import glob
import os
from multiprocessing import Pool

from tqdm import tqdm


def remove_texture(input_path):
    if input_path.endswith('obj'):
        with open(input_path, 'r') as file:
            lines = file.readlines()

        output_file = input_path.replace('.obj', '_temp.obj')
        with open(output_file, 'w') as file:
            for line in lines:
                # Skip lines starting with 'vt', 'mtllib', or 'usemtl'
                if not (line.startswith('vt') or line.startswith('mtllib') or line.startswith('usemtl')):
                    file.write(line)

        os.rename(output_file, input_path)


if __name__ == '__main__':
    obj_paths = glob.glob('../../HILO_Dataset/HILO_Objects/full/*')
    with Pool(processes=os.cpu_count()) as pool:
        for _ in tqdm(pool.imap_unordered(remove_texture, obj_paths), total=len(obj_paths)):
            pass
