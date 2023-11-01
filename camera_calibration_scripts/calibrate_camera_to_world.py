import numpy as np
import open3d as o3d

# def calibrate_minimization(A, B):

    # E = SUM{ Q[i] - (R * P[i] + T) | Q[i] - (R * P[i] + T)}

def ICP_calibration(trans_init, A, B):
    threshold = 0.2
    p2p = o3d.pipelines.registration.registration_icp(A, B, threshold, trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint())
    return p2p.transformation

def calibrate_pointclouds(A, B):
    """
    This script assumes that a dataset of 3D points in the camera frame and the corresponding
    3D points in the robot frame is provided. To generate a dataset of this format, use the 
    file save_calibration_object_data.py. Given the 3D points and their correspondences,
    this function solves for the camera extrinsics (translation and rotation) simply using least
    squares optimization. 

    The basic equation (assuming no error, which is not the case in practice) is:
        RA + t = B (where A and B are the 3D points datasets in the camera and robot frames, respectively)
    """
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B
    return R, t

def parse_calibration_folders(start_idx, stop_idx, camera):
    """
    This function parses the calibration folders and returns the 3D points in the camera frame
    and the corresponding 3D points in the robot frame. 
    """
    path = "Calibration_Data/Horizontal_Board/Cam" + str(camera)

    cam_list = []
    rob_list = []
    for i in range(start_idx, stop_idx):
        cam_filename = path + "/cam" + str(camera) + "_camframe_" + str(i) + ".npy"
        rob_filename = path + "/cam" + str(camera) + "_robframe_" + str(i) + ".npy"
        cam_list.append(np.load(cam_filename))
        rob_list.append(np.load(rob_filename))
    
    A = np.transpose(np.concatenate(cam_list, axis=0))
    B = np.transpose(np.concatenate(rob_list, axis=0))
    return A, B


if __name__ == "__main__":
    cam = 2

    # import the calibration dataset
    # A, B = parse_calibration_folders(0, 6, camera=cam)
    A, B = parse_calibration_folders(0, 15, camera=cam)

    # validate calibration
    pca = o3d.geometry.PointCloud()
    pca.points = o3d.utility.Vector3dVector(np.transpose(A))
    pca_colors = np.tile(np.array([1, 0, 0]), (len(A),1))
    pca.colors = o3d.utility.Vector3dVector(pca_colors)
    pcb = o3d.geometry.PointCloud()
    pcb.points = o3d.utility.Vector3dVector(np.transpose(B))
    pcb_colors = np.tile(np.array([0, 0, 1]), (len(B),1))
    pcb.colors = o3d.utility.Vector3dVector(pcb_colors)
    o3d.visualization.draw_geometries([pca, pcb])

    # get rotation and translation
    R, t = calibrate_pointclouds(A, B)

    # combine the rotation and translation into the rigid body transform matrix
    world_transform = np.hstack((R, t))
    world_transform = np.vstack((world_transform, np.array([0,0,0,1])))
    print("World Transform: ", world_transform)

    # validate calibration
    pca = o3d.geometry.PointCloud()
    pca.points = o3d.utility.Vector3dVector(np.transpose(A))
    pca_colors = np.tile(np.array([1, 0, 0]), (len(A),1))
    pca.colors = o3d.utility.Vector3dVector(pca_colors)
    pca.transform(world_transform)
    pcb = o3d.geometry.PointCloud()
    pcb.points = o3d.utility.Vector3dVector(np.transpose(B))
    pcb_colors = np.tile(np.array([0, 0, 1]), (len(B),1))
    pcb.colors = o3d.utility.Vector3dVector(pcb_colors)
    o3d.visualization.draw_geometries([pca, pcb])

    # # evaluate the SVD calibration
    # svd_evaluation = o3d.pipelines.registration.evaluate_registration(pca, pcb, 0.0002, world_transform)
    # print(svd_evaluation)

    # # get ICP of the calibrated transform
    # trans_init = np.eye(4)
    # icp_transform = ICP_calibration(trans_init, pca, pcb)
    
    # pca = o3d.geometry.PointCloud()
    # pca.points = o3d.utility.Vector3dVector(np.transpose(A))
    # pca_colors = np.tile(np.array([1, 0, 0]), (len(A),1))
    # pca.colors = o3d.utility.Vector3dVector(pca_colors)
    # pca.transform(icp_transform)
    # pcb = o3d.geometry.PointCloud()
    # pcb.points = o3d.utility.Vector3dVector(np.transpose(B))
    # pcb_colors = np.tile(np.array([0, 0, 1]), (len(B),1))
    # pcb.colors = o3d.utility.Vector3dVector(pcb_colors)
    # o3d.visualization.draw_geometries([pca, pcb])

    # # evaluate the ICP calibration
    # icp_evaluation = o3d.pipelines.registration.evaluate_registration(pca, pcb, 0.0002, icp_transform)
    # print(icp_evaluation)

    # print("ICP Transform: ", icp_transform)

    # save the extrinsic matrix
    np.save("Calibration_Data/Horizontal_Board/cam" + str(cam) + "_world_transform.npy", world_transform)