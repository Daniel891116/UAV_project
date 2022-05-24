from pymavlink import mavutil
import time

def set_position(master, coordi: list, boot_time):
    master.mav.set_position_global_int_send(
    int(1e3 * (time.time() - boot_time)),
    master.target_system,
    master.target_component,
    coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    type_task = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    ),
    lat_int = coordi[0],
    lon_int = coordi[1],
    alt     = coordi[2],
    )

def check_position(master, coordi:list, error: float) -> bool:
    while True:
        msg = master.recv_match()
        if not msg:
            return False
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            return (msg.lat - coordi[0]) ** 2 + (msg.lon - coordi[1]) ** 2 + (msg.alt - coordi[2]) ** 2 < error ** 2
        elif msg.get_type() == 'ALTITUDE': 
            print(msg.todict())
            return False
        elif msg.get_type() == 'SCALED_PRESSURE':
            print(msg.todict())
            return False

# Create the connection
# master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
master = mavutil.mavlink_connection('udpin:localhost:14551')

boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()
print('ok')

mission_points = [
    [0, 0, 0],
    [10, 0, 0],
    [10, 10, 0],
    [10, 10, 10],
    [0, 0, 0]
]

master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
                                    
for coordi in mission_points:
    set_position(master, coordi, boot_time)
    start_time = time.time()
    while not check_position(master, coordi, error = 5.0):
        pass
    print(f'Go to {coordi} takes {time.time() - start_time} ms')