# echo_client.py
# -*- coding:utf-8 -*-

import socket
import numpy
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math

# socket connection address and port
SERVER_IP = '192.168.137.232'
SERVER_PORT = 2204
SIZE = 512
SERVER_ADDR = (SERVER_IP, SERVER_PORT)

num = 0  # Current Target point to send Server


def arm_and_takeoff_nogps(aTargetAltitude):
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.6
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude * 0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust=thrust)
        time.sleep(0.2)


def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(yaw_rate),  # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)


def set_attitude(roll_angle=0.0, pitch_angle=0.0,
                 yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                 thrust=0.5, duration=0):
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def drone_fly(lati, longti):
    vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
    print("connected")
    arm_and_takeoff_nogps(1)

    print("Angle Positioning and move toward")  # point B to C
    loc_point = LocationGlobalRelative(lati, longti, 1)
    vehicle.simple_goto(loc_point,groundspeed=8)
    time.sleep(20)

    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(1)

    print("Close vehicle object")
    vehicle.close()
    print("Ready to leave to next base")


#   To connect socket connection
try:
    #   Client socket connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.connect(SERVER_ADDR)  # connect to server
        msg = client_socket.recv(126)  # get message from server
        print("resp from server : {}".format(msg))  # print message from server
        client_socket.close()

        msg = str(msg)
        locations = msg.split('\'')
        locations = locations[1]
        locations = locations.split('/')
        del locations[12]
        locations = list(map(float, locations))
        print(locations)

        # to make path
        i = 0
        longtitude = []
        latitude = []
        for i in range(len(locations)):
            if i % 2 == 0:
                longtitude.append(locations[i])
            else:
                latitude.append(locations[i])resp from server: {}".format(msg))  # print message from server

        for i in range(len(latitude)):
            print('lat=',latitude[i], 'long=', longtitude[i])

        abc = input("waiting..")

        num = num + 1
        point = str(num)    # convert num to string type     send 1 to server
        client_socket.send(point.encode('utf-8'))
        time.sleep(3)

        #1
        while num < 5:
            drone_fly(latitude[num], longtitude[num])

            num = num + 1
            point = str(num)    # convert num to string type     send 2 to server
            client_socket.send(point.encode('utf-8'))
            time.sleep(3)

        #5(Call back)
        vehicle.mode = VehicleMode("RTL")
        print("Completed to Base")
        print("Finish")

        point = 'arrive'
        client_socket.send(point.encode('utf-8'))   # Socket conection finish
        client_socket.close()      # close socket connection

except socket.error:    # when socket connection failed
    print("EMERGENCY LAND!!")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(1)
    print("Close vehicle object")
    vehicle.close()
finally:
    client_socket.close()
