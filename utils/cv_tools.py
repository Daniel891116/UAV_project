import numpy as np
from numpy import linalg as LA
import cv2 as cv
from typing import Union

def calCameraMotion(new_pts: np.ndarray, prev_pts: np.ndarray, intrinsic_mat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    focal_length = 0.5 * (intrinsic_mat[0, 0] + intrinsic_mat[1, 1])
    principle_point = (intrinsic_mat[0, 2], intrinsic_mat[1, 2])
    E, mask = cv.findEssentialMat(prev_pts, new_pts, focal_length, principle_point, cv.RANSAC, 0.999, 1.0)
    # print(f'Essential matrix: \n{E}')
    # print(mask)
    pass_count, R, T, mask = cv.recoverPose(E, prev_pts, new_pts, intrinsic_mat, mask)
    # print(mask)
    # print(f'Transformation: \n{R}')
    # print(f'Transpose: \n{T}')
    return R, T, mask

def check_halt(new_pts: np.ndarray, prev_pts: np.ndarray, distThreshold: float) -> bool:
    dist = np.mean(LA.norm(new_pts - prev_pts))**0.5
    print(f"error: {dist}")
    if dist > distThreshold:
        return False
    return True

def camera_update(index: int, camera_Dots, camera_pos: np.ndarray):
    camera_Dots.set_data(camera_pos[:index, 0:2].T)
    camera_Dots.set_3d_properties(camera_pos[:index, 2].T)
    return camera_Dots,

def camera_init(camera_Dots, camera_pos: np.ndarray):
    camera_Dots.set_data(camera_pos[0, 0:2].T)
    camera_Dots.set_3d_properties(camera_pos[0, 0:2].T)
    return camera_Dots,
    
def OPMotion(new_pts: np.ndarray, prev_pts: np.ndarray) -> np.ndarray:
    diff = new_pts - prev_pts
    scales = LA.norm(diff, axis = -1)
    scale_mask = detectOutlier(inputs = scales, threshold = 1.0)
    angles = calAngle(diff = diff)
    angle_mask = detectOutlier(inputs = angles, threshold = 1.0)
    mask = scale_mask * angle_mask
    diff_inlier = diff[mask == 1]
    if len(diff_inlier) != 0:
        camera_move = np.append(np.mean(diff_inlier, axis = 0), 0)
    else:
        camera_move = np.array([0, 0, 0])
    return camera_move[:, None]

def calAngle(diff: np.ndarray) -> np.ndarray:
    angles = np.zeros((diff.shape[0]))
    for i in range(diff.shape[0]):
        if diff[i][0] == 0:
            if diff[i][1] >= 0:
                angles[i] = 90.0
            else :
                angles[i] = -90.0
        elif diff[i][0] < 0:
            tan = diff[i][1] / diff[i][0]
            angles[i] = np.arctan(tan) * 180 / np.pi
            if diff[i][1] >= 0:
                angles[i] += 180.0
            else :
                angles[i] -= 180.0
    return angles

def detectOutlier(inputs: np.ndarray, threshold: float = 1.0) -> np.ndarray:
    median = np.median(inputs, axis = 0)
    std = np.std(inputs, axis = 0)
    mask = np.zeros_like(inputs)
    # print(median, std)
    for i in range(inputs.shape[0]):
        if np.abs(inputs[i] - median) <= threshold * std:
            mask[i] = 1
    return mask