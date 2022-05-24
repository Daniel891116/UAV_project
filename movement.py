from pymavlink import mavutil
from utils import change_mode
import time

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

change_mode(the_connection, "GUIDED")

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

time.sleep(10)

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(20, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 40, 0, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

msg = the_connection.recv_match(type='MISSION_REQUEST', blocking=True)
print(msg)

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(20, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 10, 0, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

# the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
#                         the_connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), int(-35.3629849 * 10 ** 7), int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

while 1:
    msg = the_connection.recv_match(
        type='LOCAL_POSITION_NED', blocking=True)
    # print(msg)