import numpy as np
import cv2 as cv

def calCameraMotion(new_pts: np.ndarray, prev_pts: np.ndarray, intrinsic_mat: np.ndarray) -> np.ndarray:
    focal_length = 0.5 * (intrinsic_mat[0, 0] + intrinsic_mat[1, 1])
    principle_point = (intrinsic_mat[0, 2], intrinsic_mat[1, 2])
    E, mask = cv.findEssentialMat(prev_pts, new_pts, focal_length, principle_point, cv.RANSAC, 0.999, 1.0)
    print(E)
    print(mask)
    return E