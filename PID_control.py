from pymavlink import mavutil
from utils import change_mode, arm, disarm, takeoff, goto_wp, arrived_wp, set_speed, get_mode, send_manual_command
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
# change_mode(master, "STABILIZE")
change_mode(master, "GUIDED")

get_mode(master)
takeoff(master, 10)
time.sleep(5)
change_mode(master, "ALT_HOLD")
get_mode(master)

vx = 0.0
vy = 0.0
vz = 0.0
control_signal = {'roll':0,'pitch':0,'throttle':500,'yaw':0}

cap = cv2.VideoCapture(0)

while True:
      ret, frame = cap.read()
      cv2.imshow('frame', frame)
      key = cv2.waitKey(1)
      if key & 0xFF == 27: # ESC
            break
      elif key & 0xFF == ord('w'):
            vx += 10
      elif key & 0xFF == ord('s'):
            vx -= 10
      elif key & 0xFF == ord('a'):
            vy += 10
      elif key & 0xFF == ord('d'):
            vy -= 10
      elif key & 0xFF == 82:
            vz += 10
      elif key & 0xFF == 84:
            vz -= 10
      
      print(f'send time: {int(1e3 * (time.time() - boot_time))}')
      control_signal['pitch'] = vy
      control_signal['roll'] = vx
      control_signal['yaw'] = vz
      print(f'control_signal:\n {control_signal}')
      send_manual_command(master, control_signal)

cap.release()
cv2.destroyAllWindows()

change_mode(master, "LAND")
control_signal = {'roll':0,'pitch':0,'throttle':0,'yaw':0}
send_manual_command(master, control_signal)
disarm(master)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
print("disarm")
