from pymavlink import mavutil
import numpy as np
import cv2 as cv
import argparse
import sys, os
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

sys.path.append(os.path.join(os.getcwd(), "../")) #append parent dir
from utils import OPMotion, camera_update
from utils import change_mode, arm, disarm, takeoff, get_mode, send_manual_command, PID

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

_maxFeature = 16

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
# ax.set_xlim3d(-1000, 1000)
# ax.set_ylim3d(-1000, 1000)
# ax.set_zlim3d(-1000, 1000)
ax.set_title('Trajectory of camera')

origin_camera_pos = np.array([[0], [0], [0]], dtype = np.float64)

camera_pos = []
PID_feedback = []
control_signal = {'roll':0,'pitch':0,'throttle':500,'yaw':0}
PID_disable = True
# ========================================================================
# Start a connection listening to a UART port
try:
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud = 57600)

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
        (master.target_system, master.target_component))
    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1)
    # master.mav.command_int_send(master.target_system, master.target_component, 0, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 1, 0, 0, 0, 0, 0, 0)
    # msg = master.recv_match(type='COMMAND_ACK', blocking=False)
    # print(msg)


    boot_time = time.time()
    print(f'boot time is {boot_time}')
    change_mode(master, "ALT_HOLD")
    arm(master)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    change_mode(master, "GUIDED")
    get_mode(master)
    print('takeoff...')
    takeoff(master, 1)
    get_mode(master)
    control_signal = {'roll':0,'pitch':0,'throttle':580,'yaw':0}
    send_manual_command(master, control_signal)
    time.sleep(1)
    change_mode(master, "ALT_HOLD")
    get_mode(master)

    xPID = PID()
    yPID = PID()

    # Take first frame and find corners in it
    ret, old_frame = cap.read()
    old_frame = cv.rotate(old_frame, cv.ROTATE_180)
    # old_frame = cv.resize(old_frame, (640, 480)) 

    # set camera intrinsic matrix
    camera_h = old_frame.shape[0]
    camera_w = old_frame.shape[1]

    old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
    # ShiTomasi corner detection
    p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
    # SIFT
    # sift = cv.SIFT_create(**SIFT_params)
    # sift_p0 = sift.detect(old_gray, None)

    # p0 = np.array(list(map(lambda keypoint: list(keypoint.pt), sift_p0)), dtype = np.float32)
    # p0 = np.expand_dims(p0, axis = 1)

    print(f'detech {len(p0)} kps')

    # Create a mask image for drawing purposes
    mask = np.zeros_like(old_frame)
    step = 0
    start_time = time.time()
    while True:
        if time.time() - start_time >= 1:
            # print(time.time(), start_time)
            p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
            # print('update kp')
            start_time = time.time()

        ret,frame = cap.read()
        if not ret:
            break
        frame = cv.rotate(frame, cv.ROTATE_180)
        # frame = cv.resize(frame, (640, 480)) 
        new_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # calculate optical flow
        # print(type(p0))
        if type(p0) != type(None) and len(p0) != 0:
            p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, new_gray, p0, None, **lk_params)
            # p1 = np.around(p1)
            # Select good points
            good_new = p1[st==1]
            good_prev = p0[st==1]
        else :
            good_new = np.array([])
            good_prev = np.array([])

        T = OPMotion(new_pts = good_new, prev_pts = good_prev)
        # PID control
        try:
            if not PID_disable:
                # update camera position
                step += 1
                origin_camera_pos += T
                xc = xPID.correct(-origin_camera_pos[0][0],P=0.2,I=1e-3, D=0)#1e-5,D=2e-1)
                yc = yPID.correct( origin_camera_pos[1][0],P=0.2,I=1e-3, D=0)#1e-5,D=2e-1)
                print(f'Input:\n x: {-origin_camera_pos[0][0]}, y:{origin_camera_pos[1][0]}')
                control_signal['pitch'] = -yc
                control_signal['roll'] = -xc
                print(f'control_signal: \n{control_signal}')
                # print(f'roll: {control_signal['roll']}, pitch: {control_signal['pitch']}')
                send_manual_command(master, control_signal)
                camera_pos.append(origin_camera_pos.copy())
                PID_feedback.append([control_signal['roll'], control_signal['pitch']])
        except:
            print('PID error')
            break
        
        # print(f'[{step}]pos:\n{T}')
        
        
        # draw the tracks
        if good_new.size != 0:
            for i,(new,old) in enumerate(zip(good_new, good_prev)):
                a,b = new.ravel()
                c,d = old.ravel()
                # mask = cv.line(mask, (int(a),int(b)),(int(c),int(d)), color[i].tolist(), 2)
                draw_frame = cv.circle(frame,(int(a),int(b)),2,color[i].tolist(),-1)
            # img = cv.add(draw_frame,mask)
            # cv.imshow('frame',img)
        else :
            draw_frame = frame
        cv.imshow('camera', draw_frame)

        k = cv.waitKey(30) & 0xff
        if k == 27:
            break
        if k == ord('s'):
            PID_disable = False
        # Now update the previous frame and previous points
        old_gray = new_gray.copy()
        p0 = good_new.reshape(-1,1,2)
        
    cap.release()
    cv.destroyAllWindows()
    # change_mode(master, "LAND")
    control_signal = {'roll':0,'pitch':0,'throttle':0,'yaw':0}
    send_manual_command(master, control_signal)
    disarm(master)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print("disarm")

    camera_pos = np.array(camera_pos)
    camera_pos = np.squeeze(camera_pos, axis = -1)
    camera_Dots = ax.plot(camera_pos[:, 0], camera_pos[:, 1], camera_pos[:, 2], marker = 'o', markersize = 6)[0]
    ani = animation.FuncAnimation(
        fig = fig, 
        func = camera_update, 
        fargs = (camera_Dots, camera_pos), 
        frames = camera_pos.shape[0], 
        interval = 4000/camera_pos.shape[0] * 2, 
        blit = True
    )
    # plt.savefig("camera_movement.pdf")
    try:
        plt.show()
    except:
        pass

    gif_save = str(input("want to save this GIF?:[y/n]"))
    if gif_save == 'y':
        print('GIF is saving...')
        ani.save('Camera_movement.gif', writer = 'pillow', fps = 1/0.08)
    else:
        pass

    PID_feedback = np.array(PID_feedback)
    fig, ax = plt.subplots(2, 1, sharex = 'all')
    ax[0].set_title('x pos control')
    # ax[0].legend(['x pos', 'roll correction'], loc = "lower right")
    PID_ax_roll = ax[0].twinx()
    ax[0].plot(-camera_pos[:, 0], 'r-o', markersize = 1)
    PID_ax_roll.plot(PID_feedback[:, 0], 'b-o', markersize = 1)
    ax[0].set_ylabel('pixel')
    PID_ax_roll.set_ylabel('roll')

    ax[1].set_title('y pos control')
    # ax[1].legend(['y pos', 'pitch correction'], loc = "lower right")
    PID_ax_pitch = ax[1].twinx()
    ax[1].plot(camera_pos[:, 1], 'r-o', markersize = 1)
    PID_ax_pitch.plot(PID_feedback[:, 1], 'b-o', markersize = 1)
    ax[1].set_xlabel('step')
    ax[1].set_ylabel('pixel')
    PID_ax_pitch.set_ylabel('pitch')
    
    plt.savefig('PID response.png')
    plt.show()

    
except KeyboardInterrupt:
    control_signal = {'roll':0,'pitch':0,'throttle':0,'yaw':0}
    disarm(master)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print("disarm3")