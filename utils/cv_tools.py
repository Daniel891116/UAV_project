import numpy as np
import cv2 as cv

def calCameraMotion(new_pts: np.ndarray, prev_pts: np.ndarray, intrinsic_mat: np.ndarray) -> (np.ndarray, np.ndarray):
    focal_length = 0.5 * (intrinsic_mat[0, 0] + intrinsic_mat[1, 1])
    principle_point = (intrinsic_mat[0, 2], intrinsic_mat[1, 2])
    E, mask = cv.findEssentialMat(prev_pts, new_pts, focal_length, principle_point, cv.RANSAC, 0.999, 1.0)
    # print(f'Essential matrix: \n{E}')
    # print(mask)
    pass_count, R, T, mask = cv.recoverPose(E, prev_pts, new_pts, intrinsic_mat, mask)
    # print(f'Transformation: \n{R}')
    # print(f'Transpose: \n{T}')
    return R, T

def camera_update(index: int, camera_Dots, camera_pos: np.ndarray):
    camera_Dots.set_data(camera_pos[:index, 0:2].T)
    camera_Dots.set_3d_properties(camera_pos[:index, 2].T)
    return camera_Dots,

def camera_init(camera_Dots, camera_pos: np.ndarray):
    camera_Dots.set_data(camera_pos[0, 0:2].T)
    camera_Dots.set_3d_properties(camera_pos[0, 0:2].T)
    return camera_Dots,
    
