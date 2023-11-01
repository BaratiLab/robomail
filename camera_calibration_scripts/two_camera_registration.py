import numpy as np

"""
This script is the working calibration code to get the transformation matrix between two cameras. Detailed instructions
for how to get the best calibration possible using this script are outlined further below. Our current camera calibration 
protocol is to use two_camera_registration.py to get the transformation matrix between cameras 2/3, 3/4 and 5/4. Then, we
use INSERT_SCRIPT_NAME and a calibration board to find the transformation matrix between camera 4 and the world frame.

GENERAL CALIBRATION INSTRUCTIONS FOR INTER-CAMERA TRANSFORMS:
(1) Place the elevated stage onto the robot workspace and optionally place an asymmetrical calibration object (such as a 
    stack of foam blocks).
(2) Place a solid white poster board slanted in the background of the scene. This provides an additional plane and assists
    the RANSAC algorithm in providing a quality coarse alignment.
(3) Run the two_camera_registration.py script and adjust the voxel size and background removal radius depending on 
    calibration quality, as these are the main parameters we can adjust depending on camera views. More information on
    parameters that were successful for previous calibration runs are provided below.

CALIBRATION PARAMETER NOTES:
(Cam 2/3) 
    - lean the white posterboard against camera 5 at approx a 60 degree angle
    - voxel size: 0.005
    - no background removal
    - ensure calibration object facing both cameras with the same face as much as possible

(Cam 3/4)
    - lean the white posterboard against camera 5, protruding into workspace a bit at approx a 60 degree angle
    - voxel size: 0.01
    - background removal radius: 1.65m
    - ensure calibration object facing both cameras with the same face as much as possible

(Cam 5/4)
    - lean the white posterboard against cameras 2/3 
    - voxel size: 0.018
    - background removal radius: 1.65m
    - ensure calibration object facing both cameras with the same face as much as possible
"""