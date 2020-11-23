import time
import sys
import threading
from pymavlink import mavutil

master = mavutil.mavink_connection("/dev/ttyAMA0", baud=57600)

master.wait_heartbeat()

mode = 'STABILIZE'

if mode not in master.mode_mapping():
    print('unknown {}'.format(mode))
    print('try:', list(master.mode_mapping().keys()))
    sys.exit(1)


def arm(arming):
    if arming == 'arm':
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,1,0,0,0,0,0,0
        )
    elif arming== 'disarm':
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
    arming == 'arm'
    thread_arm = threading.Thread(target=arm, args=(arming, ))
    thread_read_param = threading.Thread(target=read_param)
    
    thread_arm.start()
    thread_read_param.start()