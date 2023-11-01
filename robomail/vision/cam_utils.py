import numpy as np
import argparse
from autolab_core import RigidTransform
from perception import CameraIntrinsics

def get_cam_info(cam_number, alternate_path=None):
    """
    This is a helper function that imports the necessary camera info given the camera number. An
    optional input is an alternate path, which is a path to the calibration files if they are not
    located under /calib on your local machine. Note that the camera extrinsics are calculated with
    respect to each other during calibration, and need to be combined to get the global transform
    from the respective static camera to the world frame. This function should handle that automatically.
    """
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
        # TODO: get the path to be robust to getting called from any path
        path = "calib"

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
        2: path + "/realsense_camera23.tf",
        3: path + "/realsense_camera34.tf",
        4: path + "/realsense_camera4w.tf",
        5: path + "/realsense_camera54.tf"
    }

    # import camera intrinsics and extrinsics
    REALSENSE_INTRINSICS = camera_intrinsics[cam_number]
    
    # combine camera extrinsics to get the camera to world transform (transform order ABC --> multiply transforms CBA)
    if cam_number == 1 or cam_number == 4:
        # only import 1 or 4
        REALSENSE_TF = camera_extrinsics[cam_number]
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
        )
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_TF)
        args = parser.parse_args()
        realsense_extrinsics = RigidTransform.load(args.extrinsics_file_path)
        realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)

    elif cam_number == 2:
        # import 23 / 34/ 4w
        REALSENSE_TF_2 = camera_extrinsics[cam_number]
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
        )
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_TF_2)
        args = parser.parse_args()
        realsense_23 = RigidTransform.load(args.extrinsics_file_path)
        realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)

        REALSENSE_TF_3 = camera_extrinsics[3]
        parser = argparse.ArgumentParser()
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_TF_3)
        args = parser.parse_args()
        realsense_34 = RigidTransform.load(args.extrinsics_file_path)

        REALSENSE_TF_4 = camera_extrinsics[4]
        parser = argparse.ArgumentParser()
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_TF_4)
        args = parser.parse_args()
        realsense_4w = RigidTransform.load(args.extrinsics_file_path)
        
        realsense_extrinsics = realsense_4w @ realsense_34 @ realsense_23
         
    elif cam_number == 3:
        # import 34/ 4w
        REALSENSE_TF_3 = camera_extrinsics[cam_number]
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
        )
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_TF_3)
        args = parser.parse_args()
        realsense_34 = RigidTransform.load(args.extrinsics_file_path)
        realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)

        REALSENSE_TF_4 = camera_extrinsics[4]
        parser = argparse.ArgumentParser()
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_TF_4)
        args = parser.parse_args()
        realsense_4w = RigidTransform.load(args.extrinsics_file_path)
        
        realsense_extrinsics = realsense_4w @ realsense_34 

    else:
        # import 54/ 4w
        REALSENSE_TF_5 = camera_extrinsics[cam_number]
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
        )
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_TF_5)
        args = parser.parse_args()
        realsense_54 = RigidTransform.load(args.extrinsics_file_path)
        realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)

        REALSENSE_TF_4 = camera_extrinsics[4]
        parser = argparse.ArgumentParser()
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_TF_4)
        args = parser.parse_args()
        realsense_4w = RigidTransform.load(args.extrinsics_file_path)
        
        realsense_extrinsics = realsense_4w @ realsense_54 

    # get the camera serial number
    cam_serial = camera_serials[cam_number]
    return realsense_intrinsics, realsense_extrinsics, cam_serial