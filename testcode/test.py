import time
import sys
import threading
from dronekit import connect, VehicleMode
from pymavlink import mavutil

master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=57600)



master.wait_heartbeat()

connection_string = args.connect
vehicle = connect("/dev/ttyAMA0", wait_ready=True, baud=57600)
while not vehicle.is_armable:
    print("Waiting for vehicle")
    time.sleep(1)


mode = 'STABILIZE'

if mode not in master.mode_mapping():
    print('unknown {}'.format(mode))
    print('try:', list(master.mode_mapping().keys()))
    sys.exit(1)


def arm_takeoff(arming):
    if arming == 'arm':
        print("arming..")
#        master.mav.command_long_send(
#            master.target_system,
#            master.target_component,
#            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#            0,1,0,0,0,0,0,0
#        )
        vehicle.armed = True
        while not vehicle.armed:
            print("Waiting..")
            time.sleep(1)


        #temp code--dronekit
        print("taking off")
        master.simple_takeoff(1)
        time.sleep(2.5)
        print("landing")
        master.mode = VehicleMode("LAND")
        #dronekit-end

    else:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
    
        
def read_param():
    master.mav.param_request_list_send(
        master.target_system, master.target_component
    )
    while True:
        time.sleep(0.5)
        try:
            message = master.recv_match(type ='PARAM_VALUE', blocking=True).to_dict()
            print('name: %s\tvalue: %d' % (message['param_id'], message['param_value']))
        
        except Exception as error:
            print(error)
            sys.exit(0)



if __name__ == "__main__":
    arming = 'arm'
    thread_arm = threading.Thread(target=arm_takeoff, args=(arming, ))
    thread_read_param = threading.Thread(target=read_param)
    
    thread_arm.start()
    #thread_read_param.start()
