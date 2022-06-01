import numpy as np
import cv2 as cv
import argparse
import sys, os
sys.path.append(os.path.join(os.getcwd(), "../")) #append parent dir
from utils import calCameraMotion

parser = argparse.ArgumentParser()
# parser.add_argument('--image', type=str, help='path to image file')
# args = parser.parse_args()
# cap = cv.VideoCapture(args.image)
# cap = cv.VideoCapture('./slow_traffic_small.mp4')
cap = cv.VideoCapture(0)

_maxFeature = 8
focal_length_x = 111    # unit px
focal_length_y = 118.4  # unit px

# params for SIFT detection
SIFT_params = dict( nfeatures = _maxFeature,
                    nOctaveLayers = 3,
                    contrastThreshold = 0.04,
	                edgeThreshold = 10,
 	                sigma = 1.6 )

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = _maxFeature,
                       qualityLevel = 0.3,  # Parameter characterizing the minimal accepted quality(eg. 0.3 * Corner Response of best corner) of image corners
                       minDistance = 7,     # Minimum possible Euclidean distance between the returned corners
                       blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03)) # (type,max_iter,epsilon)

# Create some random colors
color = np.random.randint(0,255,(_maxFeature,3))

# Take first frame and find corners in it
ret, old_frame = cap.read()
# set camera intrinsic matrix
camera_h = old_frame.shape[0]
camera_w = old_frame.shape[1]
intrinsic_mat = np.array([[focal_length_x, 0, camera_w/2],
                          [0, focal_length_y, camera_h/2],
                          [0, 0, 1]])
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
# p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
sift = cv.SIFT_create(**SIFT_params)
sift_p0 = sift.detect(old_gray, None)

p0 = np.array(list(map(lambda keypoint: list(keypoint.pt), sift_p0)), dtype = np.float32)
p0 = np.expand_dims(p0, axis = 1)
prev_p = p0

# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)
while(1):
    ret,frame = cap.read()
    new_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # calculate optical flow
    p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None, **lk_params)
    # Select good points
    good_new = p1[st==1]
    good_prev = p0[st==1]
    calCameraMotion(new_pts = good_new, prev_pts = good_prev, intrinsic_mat = intrinsic_mat)
    # draw the tracks
    if (len(good_new) < len(prev_p)):
        print(f'loss {len(prev_p) - len(good_new)} keypoint(s)')
    for i,(new,old) in enumerate(zip(good_new, good_prev)):
        a,b = new.ravel()
        c,d = old.ravel()
        mask = cv.line(mask, (int(a),int(b)),(int(c),int(d)), color[i].tolist(), 2)
        frame = cv.circle(frame,(int(a),int(b)),5,color[i].tolist(),-1)
    img = cv.add(frame,mask)
    cv.imshow('frame',img)
    k = cv.waitKey(30) & 0xff
    if k == 27:
        break
    # Now update the previous frame and previous points
    old_gray = new_gray.copy()
    p0 = good_new.reshape(-1,1,2)
    prev_p = p0