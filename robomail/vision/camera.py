import cv2
import o3d
import numpy as np
import pyrealsense2 as rs
from cam_utils import *

class CameraClass():
    def __init__(self, cam_number, W=848, H=480):
        """
        Camera class to handle tracking of camera serials, intrinsics, extrinsics and the initialization
        pipeline under the hood. Simply inpyt the associated camera number and optionally the image
        dimensions to initialize the camera.
        """
        # camera number [1 is end-effector, 2-5 are static]
        self.cam_number = cam_number
        self.W = W
        self.H = H

        self.realsense_intrinsics, self.realsense_extrinsics, self.cam_serial = get_cam_info(cam_number)

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
        return self.realsense_extrinsics
    
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
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        depth_frame = frames.get_depth_frame().as_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        points = self.point_cloud.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, self.W, 3)  # xyz

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(color_image), o3d.geometry.Image(depth_image), convert_rgb_to_intensity=False)
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(width=self.W, height=self.H, fx=intrinsics.fx, fy=intrinsics.fy, cx=intrinsics.ppx, cy=intrinsics.ppy)
        pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)

        # skip empty frames
        if not np.any(depth_image):
            print("no depth")
            # continue

        return color_image, depth_image, pc, verts
    
