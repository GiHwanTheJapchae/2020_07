import sys
from pyparrot.Minidrone import Mambo


if __name__ == "__main__":

    mamboAddr = "D0:3A:08:89:E6:37"


#    mamboAddr = "6C:29:95:08:40:3A"

#    mamboAddr = "D0:3A:15:7D:E6:3B"
    mambo = Mambo(mamboAddr, use_wifi=True)

    print("[*] Trying to connect")
    success = mambo.connect(num_retries=3)
    print("[!] Connected : %s" % success)

    if(success):
        print("[*] Sleeping")
        mambo.smart_sleep(2)
        mambo.ask_for_state_update()
        mambo.smart_sleep(2)

        print("taking off")
        mambo.safe_takeoff(2)
        mambo.smart_sleep(2)

        print("[*] Flying direct : going forward")
        mambo.fly_direct(roll=0, pitch=10, yaw=0, vertical_movement=0)

        print("[*] Flying direct : going backward")
        mambo.fly_direct(roll=0, pitch=-10, yaw=0, vertical_movement=0)
        rint("[*] Flying direct :turning")
        mambo.fly_direct(roll=0, pitch=0, yaw=90, vertical_movement=0)
