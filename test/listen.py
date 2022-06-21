from pymavlink import mavutil

# Start a connection listening to a UDP port
# the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud = 57600)

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Once connected, use 'the_connection' to get and send messages
while True:
    msg = master.recv_match(type='ATTITUDE', blocking=True)
    print(msg)