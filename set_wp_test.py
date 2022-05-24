from pymavlink import mavutil
from pymavlink import mavwp
import time

from utils import change_mode, arm, disarm, takeoff, cmd_set_home

# Create the connection
# From topside computer

master = mavutil.mavlink_connection('udpin:localhost:14551')

master.wait_heartbeat()

wp = mavwp.MAVWPLoader()

def upload_mission(filepath):
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

if __name__ == '__main__':

    change_mode(master, "GUIDED")
    arm(master)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    # takeoff(master, 10)
    # msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    # print(msg)
    upload_mission('mission.txt')
    msg = master.recv_match(type='MISSION_ACK', blocking=True)
    print(msg)
    change_mode(master, "AUTO")
    master.mav.send(mavutil.mavlink.MAVLINK_cmd_mission_start_send())
    while 1:
        try:
            # msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            print(msg)
            time.sleep(1)
        except KeyboardInterrupt:
            break

    disarm(master)