
import socket
import numpy
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
​
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
​
def arm_and_takeoff_nogps(aTargetAltitude):
   ##### CONSTANTS #####
   DEFAULT_TAKEOFF_THRUST = 0.7
   SMOOTH_TAKEOFF_THRUST = 0.6
​
   print("Basic pre-arm checks")
   while not vehicle.is_armable:
       print(" Waiting for vehicle to initialise...")
       time.sleep(1)
​
   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED_NOGPS")
   vehicle.armed = True
​
   while not vehicle.armed:
       print(" Waiting for arming...")
       vehicle.armed = True
       time.sleep(1)
​
   print("Taking off!")
​
   thrust = DEFAULT_TAKEOFF_THRUST
   while True:
       current_altitude = vehicle.location.global_relative_frame.alt
       print(" Altitude: %f  Desired: %f" %
             (current_altitude, aTargetAltitude))
       if current_altitude >= aTargetAltitude*0.95:
           print("Reached target altitude")
           break
       elif current_altitude >= aTargetAltitude*0.6:
           thrust = SMOOTH_TAKEOFF_THRUST
       set_attitude(thrust=thrust)
       time.sleep(0.2)
​
​
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
​
​
def set_attitude(roll_angle=0.0, pitch_angle=0.0,
                yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                thrust=0.5, duration=0):
​
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
​
​
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
​
   w = t0 * t2 * t4 + t1 * t3 * t5
   x = t0 * t3 * t4 - t1 * t2 * t5
   y = t0 * t2 * t5 + t1 * t3 * t4
   z = t1 * t2 * t4 - t0 * t3 * t5
​
   return [w, x, y, z]
​
​
def angle_yaw(lat1, long1, lat2, long2):
   dLon = long2 - long1
​
   y = numpy.sin(dLon) * numpy.cos(lat2)
   x = numpy.cos(lat1) * numpy.sin(lat2) - numpy.sin(lat1) * numpy.cos(lat2) * numpy.cos(dLon)
​
   brng = numpy.atan2(y, x)
​
   brng = numpy.toDegrees(brng)
   brng = (brng + 360) % 360
   brng = 360 - brng; """ count degrees counter-clockwise - remove to make clockwise """
​
   return brng
​
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
        latitude.append(locations[i])

​
#1
arm_and_takeoff_nogps(1)
print("Turning angle and move toward")  # point A to B
set_attitude(pitch_angle=-5, thrust=0.5, duration=1)
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)
print("Close vehicle object")
vehicle.close()
print("Completed to A")
time.sleep(3)
​
#2
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
arm_and_takeoff_nogps(1)
print("Turning angle and move toward")  # point B to C
set_attitude(yaw_angle=45, pitch_angle=-5, thrust=0.5, duration=1)
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)
print("Close vehicle object")
vehicle.close()
print("Completed to B")
time.sleep(3)
​
#3
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
arm_and_takeoff_nogps(1)
print("Turning angle and move toward")  # point C to D
set_attitude(yaw_angle=135, pitch_angle=-5, thrust=0.5, duration=1)
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)
print("Close vehicle object")
vehicle.close()
print("Completed to C")
time.sleep(3)
​
#4
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
arm_and_takeoff_nogps(1)
print("Turning angle and move toward")  # point D to E
set_attitude(yaw_angle=180, pitch_angle=-5, thrust=0.5, duration=1)
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)
print("Close vehicle object")
vehicle.close()
print("Completed to D")
time.sleep(3)
​
#5(Call back)
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
arm_and_takeoff_nogps(1)
print("Turning angle and move toward")  # point E to A
set_attitude(yaw_angle=270, pitch_angle=-5, thrust=0.5, duration=1)
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)
print("Close vehicle object")
vehicle.close()
print("Completed to E")
time.sleep(3)
​
print("Completed to Base")
​
print("Finish")
#point = 'arrive'
#client_socket.send(point.encode('utf-8'))   # Socket conection finish
