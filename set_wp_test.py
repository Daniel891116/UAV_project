from pymavlink import mavutil
import time

from utils import change_mode, arm, disarm, takeoff, cmd_set_home, upload_mission

# Create the connection
# From topside computer

master = mavutil.mavlink_connection('udpin:localhost:14551')

master.wait_heartbeat()

if __name__ == '__main__':

    change_mode(master, "GUIDED")
    arm(master)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    # takeoff(master, 10)
    # msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    # print(msg)
    upload_mission(master, './mission.txt')
    msg = master.recv_match(type='MISSION_ACK', blocking=True)
    print(msg)
    change_mode(master, "AUTO")
    while 1:
        try:
            # msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            print(msg)
            time.sleep(1)
        except KeyboardInterrupt:
            break

    disarm(master)