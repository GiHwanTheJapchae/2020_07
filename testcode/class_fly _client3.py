import socket
import numpy
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time


class drone:
    vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
    

