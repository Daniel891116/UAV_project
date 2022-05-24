from pymavlink import mavutil
from utils import change_mode, arm, disarm, takeoff, goto_wp, arrived_wp
import time

wp_error = 10.0

# Start a connection listening to a UDP port
master = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))
boot_time = time.time()
print(f'boot time is {boot_time}')
change_mode(master, "GUIDED")
arm(master)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
takeoff(master, 10)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

time.sleep(10)
print(f'send time: {int(1e3 * (time.time() - boot_time))}')
goto_wp(master, int(1e3 * (time.time() - boot_time)), type = 'local', mask = 'Use_Position', position = [40, 0, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5])
arrived_wp(master, type = 'local', coordi = [40, 0, -10], error = wp_error)
goto_wp(master, int(1e3 * int(time.time() - boot_time)), type = 'local', mask = 'Use_Position', position = [40, 40, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5])
arrived_wp(master, type = 'local', coordi = [40, 40, -10], error = wp_error)
goto_wp(master, int(1e3 * int(time.time() - boot_time)), type = 'local', mask = 'Use_Position', position = [0, 40, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5])
arrived_wp(master, type = 'local', coordi = [0, 40, -10], error = wp_error)
goto_wp(master, int(1e3 * int(time.time() - boot_time)), type = 'local', mask = 'Use_Position', position = [0, 0, -10, 0, 0, 0, 0, 0, 0, 1.57, 0.5])
arrived_wp(master, type = 'local', coordi = [0, 0, -10], error = wp_error)

