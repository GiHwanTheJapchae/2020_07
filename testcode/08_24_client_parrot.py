import time
from pyparrot.Minidrone import Mambo
import socket
import numpy
import serial
import math

# socket connection address information
SERVER_IP = '127.0.0.1'  #
SERVER_PORT = 2204
SIZE = 512
SERVER_ADDR = (SERVER_IP, SERVER_PORT)

# client socket configuration
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

arduino = serial.Serial('COM10', 9600)

#   Start to connect Mambo
#    mamboAddr = "6C:29:95:08:40:3A"
#    mamboAddr = "D0:3A:15:7D:E6:3B"
mamboAddr = "D0:3A:08:89:E6:37"
mambo = Mambo(mamboAddr, use_wifi=True)
print("[*] Trying to connect")
success = mambo.connect(num_retries=3)
print("[!] Connected : %s" % success)


def taking_off_N_con(froll=0, fpitch=0, fyaw=0, fver=0):
    print("taking off")
    mambo.safe_takeoff(2)
    mambo.smart_sleep(2)

    count = ['froll', 'fpitch', 'fyaw', 'fver']
    param = [froll, fpitch, fyaw, fver]
    for i in range(1, 2):
        mambo.fly_direct(roll=froll, pitch=fpitch,
                         yaw=fyaw, vertical_movement=fver)
        mambo.smart_sleep(2)
        time.sleep(2)

    mambo.smart_sleep(2)
    mambo.safe_land(2)
    mambo.smart_sleep(2)


def get_yaw_angle(lat1, long1, lat2, long2):
    dLon = long2 - long1

    y = numpy.sin(dLon) * numpy.cos(lat2)
    x = numpy.cos(lat1) * numpy.sin(lat2) - numpy.sin(lat1) * \
        numpy.cos(lat2) * numpy.cos(dLon)

    brng = math.atan2(y, x)

    brng = numpy.degrees(brng)
    brng = (brng + 360) % 360
    """ count de60 grees counter-clockwise - remove to make clockwise """

    return 360-brng


def macro(pit, rol, ya, duration):

    if ya > 180:
        ya = ya-360

    if ya > 90:
        for i in range(ya/90):
            mambo.fly_direct(roll=0, pitch=0, yaw=90, vertical_movement=0)
        mambo.fly_direct(roll=0, pitch=0, yaw=(ya%90), vertical_movement=0)
    elif: ya < -90:
        for i in range((ya/90)*(-1)):
            mambo.fly_direct(roll=0, pitch=0, yaw=-90, vertical_movement=0)
        mambo.fly_direct(roll=0, pitch=0, yaw=-(ya % 90), vertical_movement=0)

    mambo.safe_takeoff(2)
    mambo.fly_direct(roll=0, pitch=0, yaw=ya, vertical_movement=0)
    mambo.smart_sleep(1)
    start = time.time()
    while time.time() - start < duration:
        mambo.fly_direct(roll=rol, pitch=pit, yaw=0, vertical_movement=0)
        time.sleep(0.1)

    mambo.safe_land(2)
    mambo.smart_sleep(2)


if __name__ == "__main__":

    num = 0
    i = 0

    yaw_angle = []
    while True:
        if (i == 5):
            break
        yaw_angle.append(get_yaw_angle(
            latitude[i], longtitude[i], latitude[i + 1], longtitude[i + 1]))
        i = i + 1

    if (success):
        print("[*] Sleeping")
        mambo.smart_sleep(2)
        mambo.ask_for_state_update()
        mambo.smart_sleep(2)
        # mamboTurn(roll1 r1, roll2 r2, yaw1 y1, yaw2 y2)

        print("Ready to leave to B")

        # 1     A to B
        yaw_val = yaw_angle[num]
        print(yaw_val)
        macro(pit=20, rol=10, ya=yaw_val, duration=1.7)
        num = num + 1
        point = str(num)
        arduino.write(point.encode())
        print("Ready to leave to C")

        # 2     B to C
        yaw_val = yaw_angle[num] - yaw_angle[num - 1]
        print(yaw_angle[num])
        macro(pit=20, rol=0, ya=yaw_val, duration=2)
        num = num + 1
        point = str(num)
        arduino.write(point.encode())
        print("Ready to leave to D")

        # 3     C to D
        yaw_val = yaw_angle[num] - yaw_angle[num - 1]
        print(yaw_val)
        macro(pit=20, rol=5, ya=yaw_val, duration=1.5)
        num = num + 1
        point = str(num)
        arduino.write(point.encode())
        print("Ready to leave to E")

        # 4     D to E
        yaw_val = yaw_angle[num] - yaw_angle[num - 1]
        print(yaw_val)
        macro(pit=20, rol=10, ya=yaw_val, duration=3)
        num = num + 1
        point = str(num)
        arduino.write(point.encode())
        print("Ready to leave to A(Base)")

        # 5     E to A
        yaw_val = yaw_angle[num] - yaw_angle[num - 1]
        print(yaw_val)
        macro(pit=20, rol=10, ya=yaw_val, duration=3)
        print('Mission complete!')
