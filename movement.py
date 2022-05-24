from pymavlink import mavutil
from utils import change_mode, arm, disarm, takeoff, goto_wp, get_wp_distance
import time

# Start a connection listening to a UDP port
master = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

change_mode(master, "GUIDED")
arm(master)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
takeoff(master, 10)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

time.sleep(10)

# master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(20, master.target_system,
#                         master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 40, 0, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

goto_wp(master, 20, type = 'local', mask = 'Use_Position', position = [40, 0, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5])

# master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(20, master.target_system,
#                         master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 10, 0, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

msg = master.recv_match(type='COMMAND_ACK', blocking=False)
print(msg)

# master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, master.target_system,
#                         master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(-35.3629849 * 10 ** 7), int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

while 1:
    # msg = master.recv_match(
    #     type='LOCAL_POSITION_NED', blocking=True)
    # print(msg)
    distance = get_wp_distance(master, type = 'local', coordi = [40, 0, -10])
    print(f'distance to next wp: {distance}')
