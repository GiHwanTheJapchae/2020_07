#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative

vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(1)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(35.1352281, 129.1015888, 1)
vehicle.simple_goto(point1, groundspeed=3)
#35.1352281, 129.1015888---dom
time.sleep(10)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

print("Close vehicle object")
vehicle.close()



vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

arm_and_takeoff(1)


print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(35.1352391, 129.1017158, 1)
#35.1352391,129.1017158 ---dom ipgu
vehicle.simple_goto(point1, groundspeed=3)

#print("Returning to Launch")
#vehicle.mode = VehicleMode("RTL")

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

print("Close vehicle object")
vehicle.close()
