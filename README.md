# Dataset of RGB-D Images of Object Collections from Multiple Viewpoints with Aligned High-Resolution 3D Models of Objects (HILO)

## Contents

[Description](#description)

[Dataset Location](#dataset-location)

[Contributors](#contributors)

[Requirements](#requirements)

[Installation](#installation)

[Troubleshooting](#troubleshooting)

[Citation](#citation)

## Description

We present the HILO dataset consisting of high-resolution 3D scanned models for 253 common-use objects and 32,256
multi-viewpoint RGB-D images with typically low-resolution data for 144 tabletop scenes consisting of collections of
random sets of 10 objects drawn from the set of 253 objects. The dataset provides the 6 degree of freedom (6DOF) pose
for all objects found in each of the 32,256 RGB-D images, obtained by performing precise 3D alignment of the 3D models
to the RGB-D images. The dataset also contains metadata on object mass, short text descriptor, binning into everyday use
classes, and aspect ratio and function categories, intrinsic parameter information for RGB-D sensors used in capture,
and transformations between camera poses. Object 3D models in the dataset were acquired by scanning using a tabletop 3D
scanner, and were manually inspected, cleaned, repaired, and exported as original ultra high-resolution at ~1M vertices
and simplified high-resolution meshes at ~10k vertices. To capture the multi-view RGB-D images, we established an
in-house testbed consisting of a turntable and two robotic manipulators to respectively cover azimuth angles and
elevation angles, and span a hemisphere. Images were captured using two Microsoft Azure Kinect sensors mounted at the
wrists of the robot, one per robot. We captured images over two distances forming hemispherical shells. We used in-house
software written in python to control the turntable movement, robot motion, and image capture, as well as to perform
camera calibration, processing to generate registered images and foreground masks, manual precise alignment of object
models to images, and post-capture correction of misalignments in camera transformation parameters. The dataset provides
value in enabling training and evaluation of algorithms for several tasks in computer vision, artificial intelligence (
AI), and robotics such as object completion, recognition, segmentation, high-resolution structure generation, robotic
grasp planning, and recognition of human-preferred grasp locations for human-robot collaboration.

## Dataset Location

[https://zenodo.org/records/15263380](https://zenodo.org/records/15263380)

## Contributors

Xinchao Song, Mingjun Li, Sean Banerjee, and Natasha Kholgade Banerjee

[Terascale All-sensing Research Studio](https://tars-home.github.io)

## Requirements

- Python 3.8
- NumPy
- SciPy
- OpenCV-Python
- Trimesh
- Pymeshlab
- Open3d
- Pillow
- Pyserial
- pyKinectAzure
- tqdm

## Installation

Get the code:

```bash
git clone https://github.com/Terascale-All-sensing-Research-Studio/hilo_dataset.git
cd hilo_dataset
```

Greate and activate a virtual environment using virtualenv:

```bash
virtualenv -p python3.8 .venv
source .venv/bin/activate
```

Or using Conda:

```bash
conda create -n hilo_dataset python=3.8 -y
conda activate hilo_dataset
```

Install packages via Pip:

```bash
pip install -r requirements.txt
```

## Troubleshooting

We are using `pykinect_azure` to access the Azure Kinect camera. For troubleshooting, please refer to its GitHub
repository: https://github.com/ibaiGorordo/pyKinectAzure.

## Citation
Please cite our paper in your publications if it helps your research:
```
@inproceedings{song2025hilo,
  title={HILO: A Large-Scale Heterogeneous Object Dataset for Benchmarking Robotic Grasping Approaches},
  author={Song, Xinchao and Banerjee, Sean and Banerjee, Natasha Kholgade},
  booktitle={2025 11th International Conference on Automation, Robotics, and Applications (ICARA)},
  pages={178--182},
  year={2025},
  organization={IEEE}
}
```
