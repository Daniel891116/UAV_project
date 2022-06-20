import numpy as np
from numpy import linalg as LA
import cv2 as cv
import argparse
import sys, os, time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

sys.path.append(os.path.join(os.getcwd(), "../")) #append parent dir
from utils import calCameraMotion, camera_update, check_halt

parser = argparse.ArgumentParser()
# parser.add_argument('--image', type=str, help='path to image file')
# args = parser.parse_args()
# cap = cv.VideoCapture(args.image)
# cap = cv.VideoCapture('./move.MOV')
# cap.set(3,640)
# cap.set(4,480)
cap = cv.VideoCapture(0)

_maxFeature = 8
focal_length_x = 506 # 111    # unit px
focal_length_y = 506 # 118.4  # unit px

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

# Create 3D env
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_title('Trajectory of camera')

origin_camera_pos = np.array([[0], [0], [0]], dtype = np.float64)
camera_pos = []

# Take first frame and find corners in it
ret, old_frame = cap.read()
old_frame = cv.resize(old_frame, (640, 480)) 

# set camera intrinsic matrix
camera_h = old_frame.shape[0]
camera_w = old_frame.shape[1]
scaler = np.array([camera_w, camera_h])
intrinsic_mat = np.array([[focal_length_x, 0, 0.5],
                          [0, focal_length_y, 0.5],
                          [0, 0, 1]])
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
# ShiTomasi corner detection
p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
# format: [[[x1, y1], [x2, y2]...]]
p0_norm = p0 / scaler
# SIFT
# sift = cv.SIFT_create(**SIFT_params)
# sift_p0 = sift.detect(old_gray, None)

# p0 = np.array(list(map(lambda keypoint: list(keypoint.pt), sift_p0)), dtype = np.float32)
# p0 = np.expand_dims(p0, axis = 1)
prev_p = p0
print(f'detech {len(p0)} kps')

# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)
step = 0
while(1):
    ret,frame = cap.read()
    if not ret:
        break
    step += 1
    frame = cv.resize(frame, (640, 480)) 
    new_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # calculate optical flow
    p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None, **lk_params)
    # p1 = np.around(p1)
    p1_norm = p1 / scaler

    # Select good points
    good_new = p1[st==1]
    good_prev = p0[st==1]
    good_new_norm = p1_norm[st==1]
    good_prev_norm = p0_norm[st==1]
    if (len(p0) - len(good_prev)):
        print(f'lost {len(p0) - len(good_prev)} kp')
        print(f'good_new: \n{good_new}')
        print(f'good_prev: \n{good_prev}')
        print(f'st" \n{st}')
        print(f'p0" \n{p0}')
    if check_halt(good_new, good_prev, 0.2):
        R = np.eye(3, 3)
        T_camera = np.zeros((3, 1), dtype = np.float32)
    else:
        R, T, _mask = calCameraMotion(new_pts = good_new_norm, prev_pts = good_prev_norm, intrinsic_mat = intrinsic_mat)
        T_camera = -np.dot(LA.inv(R), T)
        print(T_camera)
        T_camera = np.array(list(map(lambda t: t[0] if abs(t[0]) > 0.0 else 0.0, T_camera)), dtype = np.float32)
        T_camera = np.expand_dims(T_camera, axis = -1)

    # update camera position
    origin_camera_pos[0:2] += T_camera[0:2]
    # origin_camera_pos = np.around(origin_camera_pos)
    print(f'[{step}]pos:\n{T_camera}')
    
    #save all camera position
    camera_pos.append(origin_camera_pos.copy())
    
    # draw the tracks
    # print(f'detect {len(good_new)} points')
    # if (len(good_new) < len(prev_p)):
    #     print(f'loss {len(prev_p) - len(good_new)} keypoint(s)')
    for i,(new,old) in enumerate(zip(good_new, good_prev)):
        a,b = new.ravel()
        c,d = old.ravel()
        mask = cv.line(mask, (int(a),int(b)),(int(c),int(d)), color[i].tolist(), 2)
        frame = cv.circle(frame,(int(a),int(b)),5,color[i].tolist(),-1)
    img = cv.add(frame,mask)
    cv.imshow('frame',img)

    k = cv.waitKey(30) & 0xff
    if k == 27:
        cv.destroyAllWindows()
        break
    # Now update the previous frame and previous points
    old_gray = new_gray.copy()
    p0 = good_new.reshape(-1,1,2)
    prev_p = p0
    time.sleep(0.1)
    
camera_pos = np.array(camera_pos)
camera_pos = np.squeeze(camera_pos, axis = -1)
camera_Dots = ax.plot(camera_pos[:, 0], camera_pos[:, 1], camera_pos[:, 2], marker = 'o', markersize = 6)[0]
ani = animation.FuncAnimation(
    fig = fig, 
    func = camera_update, 
    fargs = (camera_Dots, camera_pos), 
    frames = camera_pos.shape[0], 
    interval = 1000/camera_pos.shape[0] * 2, 
    blit = True
)
# plt.savefig("camera_movement.pdf")
try:
    plt.show()
except:
    pass

# gif_save = str(input("want to save this GIF?:[y/n]"))
# if gif_save == 'y':
#     print('GIF is saving...')
#     ani.save('Camera_movement.gif', writer = 'pillow', fps = 1/0.04)
# else:
#     pass
