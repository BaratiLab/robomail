import numpy as np
import argparse
from autolab_core import RigidTransform
from perception import CameraIntrinsics

def get_cam_info(cam_number, alternate_path=None):
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
            1: path + "/realsense_ee_shifted.tf",
            2: path + "/realsense_camera2.tf",
            3: path + "/realsense_camera3.tf",
            4: path + "/realsense_camera4.tf",
            5: path + "/realsense_camera5.tf"
        }

        # import camera intrinsics and extrinsics
        REALSENSE_INTRINSICS = camera_intrinsics[cam_number]
        REALSENSE_EE_TF = camera_extrinsics[cam_number]
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
        )
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_EE_TF)
        args = parser.parse_args()

        realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
        realsense_to_ee_transform = RigidTransform.load(args.extrinsics_file_path)

        # get the camera serial number
        cam_serial = camera_serials[cam_number]
        return realsense_intrinsics, realsense_to_ee_transform, cam_serial