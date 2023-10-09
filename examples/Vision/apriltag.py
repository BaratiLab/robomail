import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector
from frankapy import FrankaArm
import robomail.vision as vis

fa = FrankaArm()

# initialize camera
W = 848
H = 480
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
cam = vis.GetPatchPose(cam_serial)
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

    bw_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # python wrapper AprilTag package
    detector = Detector(families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0)

    # camera parameters [fx, fy, cx, cy]
    cam_param = [realsense_intrinsics.fx, realsense_intrinsics.fy, realsense_intrinsics.cx, realsense_intrinsics.cy]
    detections = detector.detect(bw_image, estimate_tag_pose=True, camera_params=cam_param, tag_size=0.03)
    print("\nNumber of AprilTags: ", len(detections))

    # loop over the detected AprilTags
    for d in detections:

        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = d.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        # draw the bounding box of the AprilTag detection
        cv2.line(color_image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(color_image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(color_image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(color_image, ptD, ptA, (0, 255, 0), 2)

        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(d.center[0]), int(d.center[1]))
        cv2.circle(color_image, (cX, cY), 5, (0, 0, 255), -1)

        # get the pose of the AprilTag in the world frame
        bounds = np.array([ptA, ptB, ptC, ptD]) # define a patch of pixels (in this case AprilTag bounding box)
        object_center_point = cam.get_patch_pose(bounds, verts, d.pose_t, current_pose)
    
    cv2.imshow("Image", color_image)
    cv2.waitKey(1)
