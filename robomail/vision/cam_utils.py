import numpy as np
import argparse
from autolab_core import RigidTransform
from perception import CameraIntrinsics
import os

def rigid_transform_to_numpy(rigid_transform):
    """
    Convert a RigidTransform object to a single transformation matrix.
    """
    return np.r_[np.c_[rigid_transform._rotation, rigid_transform._translation], [[0, 0, 0, 1]]]

def get_cam_info(cam_number, alternate_path=None):
    """
    This is a helper function that imports the necessary camera info given the camera number. An
    optional input is an alternate path, which is a path to the calibration files if they are not
    located under /calib on your local machine. Note that the camera extrinsics are calculated with
    respect to each other during calibration, and need to be combined to get the global transform
    from the respective static camera to the world frame. This function should handle that automatically.
    """
    module_path = os.path.dirname(__file__)

    # dictionary of camera serials
    camera_serials = {
        1: '220222066259',
        2: '151322066099',
        3: '151322069488',
        4: '151322061880',
        5: '151322066932',
    }

    if alternate_path:
        path = alternate_path
    else:
        path = module_path + "/calib"

    # dictionary of camera intrinsics
    camera_intrinsics = {
        1: path + "/realsense_intrinsics.intr",
        2: path + "/realsense_intrinsics_camera2.intr",
        3: path + "/realsense_intrinsics_camera2.intr",
        4: path + "/realsense_intrinsics_camera4.intr",
        5: path + "/realsense_intrinsics_camera5.intr"
    }

    # dictionary of camera extrinsics
    camera_extrinsics = {
        1: path + "/realsense_ee.tf",
        2: path + "/realsense_camera2w.npy",
        3: path + "/realsense_camera3w.npy",
        4: path + "/realsense_camera4w.npy",
        5: path + "/realsense_camera5w.npy"
    }
    # camera_extrinsics = {
    #     1: path + "/realsense_ee.tf",
    #     2: path + "/realsense_camera23.npy",
    #     3: path + "/realsense_camera34.npy",
    #     4: path + "/realsense_camera4w.npy",
    #     5: path + "/realsense_camera54.npy"
    # }

    # import camera intrinsics and extrinsics
    REALSENSE_INTRINSICS = camera_intrinsics[cam_number]
    realsense_intrinsics = CameraIntrinsics.load(REALSENSE_INTRINSICS)
    
    # combine camera extrinsics to get the camera to world transform (transform order ABC --> multiply transforms CBA)
    if cam_number == 1:
        REALSENSE_TF = camera_extrinsics[cam_number]
        realsense_extrinsics = rigid_transform_to_numpy(RigidTransform.load(REALSENSE_TF))#.matrix()
    else:
        realsense_extrinsics = np.load(camera_extrinsics[cam_number])

    # # NOTE THIS IS OLD CODE FOR PREVIOUS CAMERA CONFIGURATION [WILL DELETE ONCE CONFIRM NEW SYSTEM FUNCTIONING]
    # elif cam_number == 4:
    #     # only import 4
    #     realsense_extrinsics = np.load(camera_extrinsics[cam_number])

    # elif cam_number == 2:
    #     # import 23 / 34/ 4w
    #     realsense_23 = np.load(camera_extrinsics[cam_number])
    #     realsense_34 = np.load(camera_extrinsics[3])
    #     realsense_4w = np.load(camera_extrinsics[4])
    #     realsense_extrinsics = realsense_4w @ realsense_34 @ realsense_23 
         
    # elif cam_number == 3:
    #     # import 34/ 4w
    #     realsense_34 = np.load(camera_extrinsics[cam_number])
    #     realsense_4w = np.load(camera_extrinsics[4])
    #     realsense_extrinsics = realsense_4w @ realsense_34 

    # else:
    #     # import 54/ 4w
    #     realsense_54 = np.load(camera_extrinsics[cam_number])
    #     realsense_4w = np.load(camera_extrinsics[4])
    #     realsense_extrinsics = realsense_4w @ realsense_54 

    # get the camera serial number
    cam_serial = camera_serials[cam_number]
    return realsense_intrinsics, realsense_extrinsics, cam_serial