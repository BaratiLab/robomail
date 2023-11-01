import cv2
import numpy as np
import pyrealsense2 as rs
from frankapy import FrankaArm
import robomail.vision as vis

fa = FrankaArm()

# initialize camera
W = 848 # 1280
H = 480 # 720
cam_serial = '151322061880'

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_device(cam_serial)
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)

print("[INFO] start streaming...")
pipeline.start(config)

aligned_stream = rs.align(rs.stream.color) # alignment between color and depth
point_cloud = rs.pointcloud()

# get the camera intrinsics for that camera
cam = vis.GetPixelPose(cam_serial)
realsense_intrinsics, realsense_to_ee_transform = cam.get_intrinsics_extrinsics()

while True:
    current_pose = fa.get_pose()
    frames = pipeline.wait_for_frames()
    frames = aligned_stream.process(frames)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame().as_depth_frame()

    points = point_cloud.calculate(depth_frame)
    verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, W, 3)  # xyz
    
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())

    # skip empty frames
    if not np.any(depth_image):
        print("no depth")
        # continue

    print("\n[INFO] found a valid depth frame")
    color_image = np.asanyarray(color_frame.get_data())

    # define arbitrary pixels
    i, j = 200, 400

    object_center_point = cam.get_pixel_pose(i, j, verts, current_pose)
    print("\nPixel Pose: ", object_center_point)
    
    cv2.imshow("Image", color_image)
    cv2.waitKey(1)
