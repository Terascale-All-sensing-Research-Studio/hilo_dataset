# Dataset of RGB-D Images of Object Collections from Multiple Viewpoints with Aligned High-Resolution 3D Models of Objects (HILO)

## Contents

[Description](#description)

[Contributors](#contributors)

## Description

We present the HILO dataset consisting of high-resolution 3D scanned models for 253 common-use objects and 32,256 RGB-D multi-viewpoint images with typically low-resolution data for 144 tabletop scenes consisting of collections of random sets of 10 objects drawn from the set of 253 objects. The dataset provides the 6 degree of freedom (6DOF) pose for all objects found in each of the 32,256 RGB-D images, obtained by performing precise 3D alignment of the 3D models to the RGB-D images. The dataset also contains metadata on object mass, short text descriptor, binning into everyday use classes, and aspect ratio and function categories, intrinsic parameter information for RGB-D sensors used in capture, and transformations between camera poses. Object 3D models in the dataset were acquired by scanning using a tabletop 3D scanner, and were manually inspected, cleaned, repaired, and exported as original ultra high-resolution at ~1M vertices and simplified high-resolution meshes at ~10k vertices. To capture the multi-view RGB-D images, we established an in-house testbed consisting of a turntable and two robotic manipulators to respectively cover azimuth angles and elevation angles, and span a hemisphere. Images were captured using two Microsoft Azure Kinect sensors mounted at the wrists of the robot, one per robot. We captured images over two distances forming hemispherical shells. We used in-house software written in python to control the turntable movement, robot motion, and image capture, as well as to perform camera calibration, processing to generate registered images and foreground masks, manual precise alignment of object models to images, and post-capture correction of misalignments in camera transformation parameters. The dataset provides value in enabling training and evaluation of algorithms for several tasks in computer vision, artificial intelligence (AI), and robotics such as object completion, recognition, segmentation, high-resolution structure generation, robotic grasp planning, and recognition of human-preferred grasp locations for human-robot collaboration.   

## Dataset Location
[https://zenodo.org/records/15263380](https://zenodo.org/records/15263380)

## Contributors
Xinchao Song, Mingjun Li, Sean Banerjee, and Natasha Kholgade Banerjee

[Terascale All-sensing Research Studio](https://tars-home.github.io)
