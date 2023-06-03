"""
Shoud be executed on RPi3 startup 

"""

import time
import threading
import subprocess

from src.commands import MyVehicle, connect_vehicle
from src.droneQueue import DroneQueue

# Connect to vehicle on startup
vehicle = connect_vehicle()


def start_receiver():
    # Start receiver.py as a subprocess
    subprocess.call(["python", "./src/receiver.py"])


# Start the receiver script in a separate thread
receiver_thread = threading.Thread(target=start_receiver)
receiver_thread.start()

# Alais for commands
functions = {
    "arm_and_takeoff": MyVehicle.arm_and_takeoff,
    "takeoff": MyVehicle.arm_and_takeoff,
    "fly": MyVehicle.fly,
    "yaw": MyVehicle.yaw
}

# Initializing queue
queue = DroneQueue()


# The following is not working yet because I can't run vehicle.func since func isn't
while True:
    # Check if drone is currently idle
    if not queue.current_command:
        queue.next_command()
        time.sleep(0.1)
    else:
        func_name = queue.current_command["command"]
        if func_name in functions:
            func = functions[func_name]
            args = {k: v for k, v in queue.current_command.items() if k != 'command'}

            print(f"{func_name} returned: {getattr(vehicle, func_name)(**args)}")

            queue.current_command = None

        # WIERD DYNAMIC PROGRAMMING !!! NOT SAFE AT ALL !!!
        # func_name = queue.current_command["command"]
        # if func_name in globals():
        #     func_args = ', '.join([f"{k}='{v}'" for k, v in a.items() if k != 'command'])
        #     exec(f"{func_name}({func_args})")


# vehicle.groundspeed = 1
# vehicle.arm_and_takeoff()   # May take a while. 36 seconds to become armable on sitl (simulation)
# vehicle.fly("north", 5)
# vehicle.yaw(50)
# vehicle.fly("left", 3)
# vehicle.land()
# vehicle.close()
