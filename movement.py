from pymavlink import mavutil
from utils import change_mode, arm, disarm, takeoff, goto_wp, arrived_wp, cmd_set_home, get_mode
import time

wp_error = 50

# Start a connection listening to a UDP port
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud = 57600)
# master = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1)
boot_time = time.time()
print(f'boot time is {boot_time}')

# msg = master.recv_match("GOLBAL_POSITION_INT", blocking=True)
# cmd_set_home(master, [msg.lat/10E7, msg.lon/10E7], 0)

change_mode(master, "ALT_HOLD")
get_mode(master)
arm(master)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
change_mode(master, "GUIDED")
get_mode(master)
takeoff(master, 1)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

time.sleep(5)
print(f'send time: {int(1e3 * (time.time() - boot_time))}')
goto_wp(master, int(1e3 * int(time.time() - boot_time)), type = 'global', mask = 'Use_Position', position = [230312448, 1202267219, 1, 0, 0, 0, 0, 0, 0, 1.57, 0.5])
arrived_wp(master, type = 'global', coordi = [230312448, 1202267219], error = wp_error)
goto_wp(master, int(1e3 * int(time.time() - boot_time)), type = 'global', mask = 'Use_Position', position = [230312467, 1202267508, 1, 0, 0, 0, 0, 0, 0, 1.57, 0.5])
arrived_wp(master, type = 'global', coordi = [230312467, 1202267508], error = wp_error)
goto_wp(master, int(1e3 * int(time.time() - boot_time)), type = 'global', mask = 'Use_Position', position = [230312195, 1202267558, 1, 0, 0, 0, 0, 0, 0, 1.57, 0.5])
arrived_wp(master, type = 'global', coordi = [230312195, 1202267558], error = wp_error)
goto_wp(master, int(1e3 * int(time.time() - boot_time)), type = 'global', mask = 'Use_Position', position = [230312166, 1202267271, 1, 0, 0, 0, 0, 0, 0, 1.57, 0.5])
arrived_wp(master, type = 'global', coordi = [230312166, 1202267271], error = wp_error)

time.sleep(5)

change_mode(master, "LAND")
disarm(master)
print("disarm")
