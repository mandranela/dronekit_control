from commands import MyVehicle, connect_vehicle
from droneQueue import DroneQueue
from communication import get_new_commands

# Alais for commands
functions = {
    "connect": connect_vehicle,
    "connect_vehicle": connect_vehicle,
    "arm_and_takeoff": MyVehicle.arm_and_takeoff,
    "takeoff": MyVehicle.arm_and_takeoff,
    "fly": MyVehicle.fly,
    "yaw": MyVehicle.yaw
}

queueDict = \
[
    {
        "command": "connect",
        # "connection_string": "tcp:127.0.0.1:5760"
    },
    {
        "command": "arm_and_takeoff"
    },
    {
        "command": "fly",
        "direction": "north",
        "distance": 5
    },
    {
        "command": "yaw",
        "angle": 50
    },
    {
        "command": "fly",
        "direction": "left",
        "distance": 6
    },
    {
        "command": "mode",
        "mode": "RTH"
    },
    {
        "command": "land"
    },
    {
        "command": "close"
    }
]

queue = DroneQueue(queue=queueDict)

vehicle = connect_vehicle()
vehicle.groundspeed = 1
vehicle.arm_and_takeoff()   # May take a while. 36 seconds to become armable on sitl (simulation)
vehicle.fly("north", 5)
vehicle.yaw(50)
vehicle.fly("left", 3)
vehicle.land()
vehicle.close()



# The following is not working yet because I can't run vehicle.func since func isn't 
'''
while True:
    # Check for new commands, add them to the queue if they exist
    new_commands = get_new_commands()
    if new_commands:
        queue.add_to_queue(new_commands)

    # Check if drone is currently idle
    if queue.get_current_command() == None:
        queue.next_command()

        func_name = queue.current_command["command"]
        if func_name in functions:
            func = functions[func_name]
            args = {k: v for k, v in queue.current_command.items() if k != 'command'}

            if func == connect_vehicle:
                vehicle = func(**args)
            else:
                print(f"{func_name} returned: {func(**args)}")

            queue.current_command = None


        # WIERD DYNAMIC PROGRAMMING !!! NOT SAFE AT ALL !!!
        # func_name = queue.current_command["command"]
        # if func_name in globals():
        #     func_args = ', '.join([f"{k}='{v}'" for k, v in a.items() if k != 'command'])
        #     exec(f"{func_name}({func_args})")


'''   

