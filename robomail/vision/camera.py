import cv2
import argparse
import numpy as np
import pyrealsense2 as rs
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics

class CameraClass():
    def __init__(self, cam_number, W=848, H=480):
        """
        Camera class to handle tracking of camera serials, intrinsics, extrinsics and the initialization
        pipeline under the hood. Simply inpyt the associated camera number and optionally the image
        dimensions to initialize the camera.
        """
        # camera number [1 is end-effector, 2-5 are static]
        self.cam_number = cam_number

        # dictionary of camera serials
        camera_serials = {
            1: '220222066259',
            2: '151322066099',
            3: '151322069488',
            4: '151322061880',
            5: '151322066932',
        }

        # dictionary of camera intrinsics
        camera_intrinsics = {
            1: "calib/realsense_intrinsics.intr",
            2: "calib/realsense_intrinsics_camera2.intr",
            3: "calib/realsense_intrinsics_camera2.intr",
            4: "calib/realsense_intrinsics_camera4.intr",
            5: "calib/realsense_intrinsics_camera5.intr"
        }

        # dictionary of camera extrinsics
        camera_extrinsics = {
            1: "calib/realsense_ee_shifted.tf",
            2: "calib/realsense_camera2.tf",
            3: "calib/realsense_camera3.tf",
            4: "calib/realsense_camera4.tf",
            5: "calib/realsense_camera5.tf"
        }

        # import camera intrinsics and extrinsics
        REALSENSE_INTRINSICS = camera_intrinsics[self.cam_number]
        REALSENSE_EE_TF = camera_extrinsics[self.cam_number]
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
        )
        parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_EE_TF)
        args = parser.parse_args()

        self.realsense_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
        self.realsense_to_ee_transform = RigidTransform.load(args.extrinsics_file_path)

        # get the camera serial number
        self.cam_serial = camera_serials[self.cam_number]

        # initialize the camera pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.cam_serial)
        config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

        print("[INFO] start streaming...")
        self.pipeline.start(config)

        self.aligned_stream = rs.align(rs.stream.color) # alignment between color and depth
        self.point_cloud = rs.pointcloud()

    def _get_cam_intrinsics(self):
        return self.realsense_intrinsics
    
    def _get_cam_extrinsics(self):
        return self.realsense_to_ee_transform
    
    def _get_cam_number(self):
        return self.cam_number
    
    def _get_cam_serial(self):
        return self.cam_serial
    
    def _get_next_frame(self):
        """
        Returns the color image, depth image, point cloud and verts of the next frame.
        """
        frames = self.pipeline.wait_for_frames()
        frames = self.aligned_stream.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame().as_depth_frame()

        points = self.point_cloud.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, W, 3)  # xyz
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())

        # skip empty frames
        if not np.any(depth_image):
            print("no depth")
            # continue

        print("\n[INFO] found a valid depth frame")
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image, points, verts
    
