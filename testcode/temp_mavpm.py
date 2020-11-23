import time
import sys
import threading
from dronekit import connect, VehicleMode
from pymavlink import mavutil

class drone:

    con_loc = "/dev/ttyAMA0"
    con_baud = 57600

    master = mavutil.mavink_connection("/dev/ttyAMA0", baud=57600)

    def __init__(self, master, vehicle):
        self.con_loc = "/dev/ttyAMA0"
        self.con_baud = 57600
        


    def arm(self, arming):
        if arming == 'arm':
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
        elif arming == 'disarm':
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )


    def cont(self, roll, pitch, yaw):
        master.mav.manual_control_send(
            master.target_system,
            pitch, roll, 500, yaw, 0
        )

    def read_param(self):
        master.mav.param_request_list_send(
            master.target_system, master.target_component

        )
        #https://mavlink.io/en/messages/common.html#MANUAL_CONTROL


        while True:
            time.sleep(0.5)
            #try:
                #message = master.recv_match(
                #    type='PARAM_VALUE', blocking=True).to_dict()
                #print('name: %s\tvalue: %d' %
                #    (message['param_id'], message['param_value']))
            if not msg:
                continue
            if msg.get_type() =='HEARTBEAT':
                print("\n\n*****Got message: %s*****" % msg.get_type())
                print("Message: %s" % msg)
                print("\nAs dictionary: %s" % msg.to_dict())
                # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
                print("\nSystem status: %s" % msg.system_status)

            #except Exception as error:
            #    print(error)
            #    sys.exit(0)
