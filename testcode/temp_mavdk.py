from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import temp_comm
#import argparse
#parser = argparse.ArgumentParser()
#parser.add_argument('--connect', default='127.0.0.1:14550')
#args = parser.parse_args()

# Connect to the Vehicle
#print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

# Function to arm and then takeoff to a user specified altitude


def arm_and_takeoff(aTargetAltitude):

  print("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print("vehicle initialize...")
    time.sleep(1)

  vehicle.mode = VehicleMode("GUIDED_NOGPS")
  vehicle.armed = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Take off")
  vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    #Break and return from function just below target altitude.
    if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
      print("Reached target altitude")
      break
    time.sleep(1)


def send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # type_mask
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # m/s
        0, 0, 0,  # x, y, z acceleration
        0, 0)
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def set_pitch(pitch):
    msg = vehicle.message_factory.set_attitude_target_encode(
      0,
      0,  # target system
      0,  # target component
      0b11100010,  # type mask
      [.9438, 0, 0, .17364],  # q
      0,  # body roll rate
      pitch,  # body pitch rate
      0,  # body yaw rate
      0)  # thrust
    veh1.send_mavlink(msg)

def set_yaw(heading, relative=False):
  if relative:
    is_relative=1
  else:
    is_relative=0

    msg = vehicle.message_factory.command_long_encode(0,0,
      mavutil.mavlink.MAV_CMD_CONDITION_YAW,
      0,
      heading,
      0,1,is_relative,0,0,0)
    vehicle.send_mavlink(msg)


def get_location():
  connection = comm('127.0.0.1', 8080)
  geolist = connection.conn_getd()
  for i in range(len(geolist)):
    if i % 2 == 0:
        latitude = locations[i]
    else:
        longtitude = locations[i]
  connection.conn_ed()






arm_and_takeoff(1)
time.sleep(10)
print("land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
