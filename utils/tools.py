from re import M
from pymavlink import mavutil
from pymavlink import mavwp
import sys
import time
import numpy as np
from numpy import linalg as LA

def change_mode(master,mode):
#    mode = 'STABILIZE'
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
        mode_id
    )

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

def arm(master):
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
        0, 
        1, 0, 0, 0, 0, 0, 0)

def disarm(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

def takeoff(master, altitude = 10):
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
        0, 
        0, 0, 0, 0, 0, 0, altitude)

def cmd_set_home(master, home_location, altitude):
    print(f'setting home, system:{master.target_system}, component:{master.target_component}')
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        home_location[0], # lat
        home_location[1], # lon
        altitude) 

def upload_mission(master, filepath):
    wp = mavwp.MAVWPLoader()
    home_location = None
    home_altitude = None

    with open(filepath) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:   
                linearray=line.split('\t')
                # ln_seq = int(linearray[0])
                ln_seq = i - 1
                ln_current = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_x=float(linearray[8])
                ln_y=float(linearray[9])
                ln_z=float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                if(i == 1):
                    home_location = (ln_x,ln_y)
                    home_altitude = ln_z
                p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,
                                                                ln_command,
                                                                ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                wp.add(p)
                    
    cmd_set_home(master, home_location, home_altitude)
    msg = master.recv_match(type = ['COMMAND_ACK'],blocking = True)
    print(msg)
    print(f'Set home location: {home_location[0], home_location[1], home_altitude}')
    time.sleep(1)

    #send waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True)
        print(msg)
        if msg.seq != i:
            i -= 1
            continue
        master.mav.send(wp.wp(msg.seq))
        print(f'Sending waypoint {msg.seq}')  

def goto_wp(master, sys_time: int, type: str, mask: str = 'Use_Position', position: list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]):

    if mask == 'Use_Position':
        _mask = int(0b110111111000)
    elif mask == 'Use_Velocity':
        _mask = int(0b110111000111)
    elif mask == 'Use_Acceleration':
        _mask = int(0b110000111111)
    elif mask == 'Use_Pos+Vel':
        _mask = int(0b110111000000)
    elif mask == 'Use_Pos+Vel+Accel':
        _mask = int(0b110000000000)
    elif mask == 'Use_Yaw':
        _mask = int(0b10011111111)
    elif mask == 'Use_Yaw_Rate':
        _mask = int(0b01011111111)
    else:
        raise TypeError('Undefined mask type')

    if type == 'local':
        frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            sys_time,
            master.target_system,
            master.target_component,
            frame, 
            _mask, 
            position[0],
            position[1], 
            position[2],
            position[3],
            position[4],
            position[5],
            position[6],
            position[7],
            position[8],
            position[9],
            position[10])
        )

    elif type == 'global':
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            sys_time,
            master.target_system,
            master.target_component,
            frame, 
            _mask, 
            position[0],
            position[1], 
            position[2],
            position[3],
            position[4],
            position[5],
            position[6],
            position[7],
            position[8],
            position[9],
            position[10])
        )
    
    else:
        raise TypeError('Undefined frame type')

    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

def get_wp_distance(master, type: str, coordi: list) -> float:
    if type == 'local':
        _type = 'LOCAL_POSITION_NED'
        print(msg)
        target = np.array([msg.x, msg.y, msg.z])
    elif type == 'global':
        _type = 'GLOBAL_POSITION_INT'
        msg = master.recv_match(type=_type, blocking=True)
        print(msg)
        target = np.array([msg.lat, msg.lon, msg.alt])
    else:
        raise TypeError('Undefined frame type')

    now_position = np.array(coordi)
    return LA.norm(now_position - target)