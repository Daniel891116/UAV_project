"""
Example of how to Arm and Disarm an Autopilot with pymavlink
"""
# Import mavutil
from __future__ import print_function
from pymavlink.dialects.v20 import common as mavlink2
import time
from pymavlink import mavutil
import cv2
import numpy as np
from numpy import sin,cos
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


            
def change_mode(master,mode):
#     mode = 'STABILIZE'
    print(f'change flight mode to {mode}')

    # Check if mode is available
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)

    # Get mode ID
    mode_id = master.mode_mapping()[mode]
    # Set new mode.
    # master.mav.command_long_send(
    #    master.target_system, master.target_component,
    #    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    #    0, mode_id, 0, 0, 0, 0, 0) or:
    # master.set_mode(mode_id) or:
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    while True:
        # Wait for ACK command
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Check if command in the same in `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
            continue

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break


def rotate(x,y,theta):
    return x*cos(theta)-y*sin(theta),x*sin(theta)+y*cos(theta)

def key_event(key):
    global master,Armed,control_signal
    if key == 27: # ESC
        return
    elif key & 0xFF == ord('c'):
        print('c')
        # master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
        master = mavutil.mavlink_connection('udpin:localhost:14551')
        # Wait a heartbeat before sending commands
        master.wait_heartbeat()
        print('ok')
    elif key & 0xFF == ord('t'):
        print('t')
        # Arm
        master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
        print("Waiting for the vehicle to arm")
        master.motors_armed_wait()
        Armed = True
        print('Armed!')
    elif key & 0xFF == ord('l'):
        print('l')
        Armed = False
        
        control_signal['throttle'] = 0
        send_command(master,control_signal)
        time.sleep(0.1)
        send_command(master,control_signal)
        time.sleep(0.1)
        
        master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

        # wait until disarming confirmed
        print('Disarm!')
        master.motors_disarmed_wait()
        print('Disarm!')
    elif key & 0xFF == ord('w'):
        print('w')
        control_signal['pitch'] = control_signal['pitch'] - 40
    elif key & 0xFF == ord('a'):
        print('a')
        control_signal['roll'] = control_signal['roll'] + 40
    elif key & 0xFF == ord('s'):
        print('s')
        control_signal['pitch'] = control_signal['pitch'] + 40
    elif key & 0xFF == ord('d'):
        print('d')
        control_signal['roll'] = control_signal['roll'] - 40
    elif key == 82: # up
        print('up arrow')
        control_signal['throttle'] = control_signal['throttle'] + 40
    elif key == 84: # down
        print('down arrow')
        control_signal['throttle'] = control_signal['throttle'] - 50
    elif key & 0xFF == ord('r'):
        print('R','Restart!')
        master.reboot_autopilot()
    
    return control_signal


def request_data(master):
    global x_origin,y_origin,z_origin,rotx,roty,x,y,z,roll,pitch,yaw,VFR_HUD_z,VFR_HUD_z_origin
    master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
    
    msg = master.recv_match(blocking=False)
    
    if not msg:
#         print('no msg')
        return rotx,roty,z,roll,pitch,yaw,VFR_HUD_z
    
    
    # handle the message based on its type
    msg_type = msg.get_type()
#     print(msg_type)
    if msg_type == "ATTITUDE":
        roll = msg.roll*180/np.pi
        pitch = msg.pitch*180/np.pi
        yaw = msg.yaw*180/np.pi
        
    if msg_type == "GLOBAL_POSITION_INT":
#         print(msg)
        if x_origin == 0:
            x_origin = msg.lat
            y_origin = msg.lon
            z_origin = msg.alt
        x = msg.lat - x_origin
        y = msg.lon - y_origin
        z = (msg.alt - z_origin)/10
        
        rotx,roty = rotate(x,y,0.2+(np.pi*3/4))
    if msg_type == "VFR_HUD":
        if VFR_HUD_z_origin == 0:
            VFR_HUD_z_origin = msg.alt*100
#         print(msg)
#         print(msg.alt)
        VFR_HUD_z = (msg.alt*100) - VFR_HUD_z_origin
#     if msg_type == "GPS_RAW_INT":
#         print(msg)
    
#     print(rotx,roty,z,roll,pitch,yaw)
    return rotx,roty,z,roll,pitch,yaw,VFR_HUD_z
#     return rotx,roty,VFR_HUD_z/100,roll,pitch,yaw
            

def send_command(master,controlsignal):
    master.mav.manual_control_send(
                master.target_system,
                int(np.clip(controlsignal['pitch'],-1000,1000)),
                int(np.clip(controlsignal['roll'],-1000,1000)),
                int(np.clip(controlsignal['throttle'],0,1000)),
                int(np.clip(controlsignal['yaw'],-1000,1000)),
                0)
    
class PID():
    def __init__(self,setpoint = 0):
        self.accumulate_error = 0
        self.setpoint = setpoint
        self.n = 100
        self.data_record = [setpoint for _ in range(self.n)]
        self.last_time = time.time()
    def P(self,data,Kp):
        return (data-self.setpoint)*Kp
    def I(self,data,Ki):
        self.accumulate_error += (data-self.setpoint)*Ki*(time.time()-self.last_time)
        self.last_time = time.time()
        return self.accumulate_error
    def D(self,data,Kd):
        d1 = (self.data_record[-1]-self.data_record[-(self.n//2)])
        d2 = (self.data_record[-1]-self.data_record[0])/2
        return (d1+d2)*Kd/2
    def calculate_PID(self,data,Kp,Ki,Kd):
        return self.P(data,Kp),self.I(data,Ki),self.D(data,Kd)
    def correct(self,data,P,I,D):
        self.data_record.pop(0)
        self.data_record.append(data)
        P,I,D = self.calculate_PID(data,P,I,D)
        #print('P',P)
        #D = min(max(D,-35),35)
        #print('P',P,'D',D)
        output = P+I+D
        return output
    
        

def main():
    global master,x_origin,y_origin,z_origin,x,y,z,rotx,roty,space,roll,pitch,yaw,Armed,control_signal,VFR_HUD_z,VFR_HUD_z_origin
    last_time = time.time()
    master = None
    x_origin,y_origin,z_origin,VFR_HUD_z_origin = 0,0,0,0
    x,y,z = 0,0,0
    rotx,roty = 0,0
    VFR_HUD_z = 0
    space = 150
    roll,pitch,yaw = 0,0,0
    Armed = False
    control_signal = {'roll':0,'pitch':0,'throttle':0,'yaw':0}
    xPID = PID()
    yPID = PID()
    zPID = PID(setpoint = 100)
    
    
    
    # Create the connection
    # master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
    master = mavutil.mavlink_connection('udpin:localhost:14551')
    # Wait a heartbeat before sending commands
    master.wait_heartbeat()
    print('connection ok')
#     master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
    change_mode(master,mode = 'STABILIZE')
#     change_mode(master,mode = 'ALT_HOLD')
    time.sleep(2)
    
    last_count = 0
    count = 0
    while True:
        count += 1
        key = cv2.waitKey(1)
        
        
        rotx,roty,z,roll,pitch,yaw,VFR_HUD_z = request_data(master)
#         print(f'state\tlat:{rotx:.1f},lng:{roty:.1f},alt:{z}  \troll:{(roll):.1f},pitch:{(pitch):.1f},yaw:{(yaw):.1f}')
        
        xc = xPID.correct(rotx,P=0.2,I=1e-5,D=2e-1)
        yc = yPID.correct(roty,P=0.2,I=1e-5,D=2e-1)
        zc = zPID.correct(z   ,P=1.0,I=0,D=0)
        
        control_signal['pitch'] = xc + 20
        control_signal['roll'] = -yc
#         control_signal['throttle'] = -zc
        
#         print(xc,yc,zc)

        
        if key == 27: # ESC
            break
        elif key & 0xFF == ord('i'):
            x_origin,y_origin,z_origin,VFR_HUD_z_origin = 0,0,0,0
        else:
            control_signal = key_event(key)
            
        
        if Armed or True:
            send_command(master,control_signal)
            
        if (time.time() - last_time) > 0.5:
            last_time = time.time()
            print(control_signal)
            # print(f'update rate:{(count-last_count)/0.5}Hz')
            last_count = count
            print(f'state\trotx:{rotx:.1f},roty:{roty:.1f},alt:{z}  \troll:{(roll):.1f},pitch:{(pitch):.1f},yaw:{(yaw):.1f}')
            # print(VFR_HUD_z)
            canvas = np.zeros((space,space,3))+255
            draw_pos = (0,int((rotx*0.7+space)//2))
#             draw_pos = (int((roty*0.7+space)//2),int((rotx*0.7+space)//2))
            cv2.circle(canvas,draw_pos, int(max(2,z//160)), (0,0,0), -1)
            cv2.circle(canvas,(int(space//2),int(space//2)), 4, (255,255,0), -1)
            cv2.line(canvas,draw_pos,(int(draw_pos[0]-yc*0.4),int(draw_pos[1]-xc*0.4)),(255,0,0),2)
            cv2.imshow('position',canvas)
    
            
if __name__ == '__main__':
    main()
        
            
        






        #if msg_type == "AHRS2":
        #    print(f'AHRS2\talt:{msg.altitude},lat:{msg.lat},lng:{msg.lng}')


# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].
#master.mav.manual_control_send(
#    master.target_system,
#    500,
#    -500,
#    250,
#    500,
#    0)