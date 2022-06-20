from pymavlink import mavutil
from utils import change_mode, arm, disarm, takeoff, goto_wp, arrived_wp, set_speed, get_mode
import time
import cv2

# Start a connection listening to a UDP port
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
# change_mode(master, "STABILIZE")
arm(master)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
change_mode(master, "STABILIZE")
change_mode(master, "GUIDED")

get_mode(master)
takeoff(master, 10)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

time.sleep(5)
vx = 0.0
vy = 0.0
vz = 0.0
cap = cv2.VideoCapture(0)

while True:
      ret, frame = cap.read()
      cv2.imshow('frame', frame)
      key = cv2.waitKey(1)
      if key & 0xFF == 27: # ESC
            break
      elif key & 0xFF == ord('w'):
            vx += 0.1
      elif key & 0xFF == ord('s'):
            vx -= 0.1
      elif key & 0xFF == ord('a'):
            vy += 0.1
      elif key & 0xFF == ord('d'):
            vy -= 0.1
      elif key & 0xFF == 82:
            vz += 0.1
      elif key & 0xFF == 84:
            vz -= 0.1
      
      print(f'send time: {int(1e3 * (time.time() - boot_time))}')
      set_speed(master, int(1e3 * int(time.time() - boot_time)), vx, vy, vz)
      print(f'Current Speed:\nvx: {vx}\nvy: {vy}\nvz: {vz}')

cap.release()
cv2.destroyAllWindows()

change_mode(master, "LAND")
disarm(master)
print("disarm")
