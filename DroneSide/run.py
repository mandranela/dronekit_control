"""
Shoud be executed on RPi3 startup. 
Will run reciever.py, connect to drone, execute commands.
Callback answers: success - vehicle сompleted command, ongoing - vehicle is executing command, failure - command cannot be finished.
"""

import os
import json
import time
import threading
import subprocess

from src import queue
from src.commands import MyVehicle, connect_vehicle

QUEUE_PATH = "./src/queue.txt"

# Start receiver.py as a subprocess
def start_receiver():
    subprocess.call(["python", "./src/receiver.py"])

# Start the receiver.py in a separate thread
receiver_thread = threading.Thread(target=start_receiver)
receiver_thread.start()

# Connect to vehicle on startup
vehicle = connect_vehicle()

# Alais for commands. Keys are declared function's names and values are recieved command values from sender. Exmaple: {"command": "fly"}
functions = {
    "arm": ["arm"],
    "arm_and_takeoff": ["takeoff", "arm_and_takeoff"],
    "fly": ["fly", "move"],
    "yaw": ["yaw", "rotate"],
    "change_mode": ["mode", "change mode", "change_mode", "mode change", "mode_change"],
}

# Initialize current command variable, pause status and callback for commands
pause_status = False
current_command = None
callback_command = None

# Main loop
while True:
    # Checking for queue special commands: pause, resume, skip, clear. 
    if queue.check_command('pause'):
        if not current_command or not callback_command:
            print("Pause: No command is currently executing. Skipping command.")
        else:
            # TODO: Implement pause command. Pause current command with capability to comtinue it in future with resume command.
            pause_status = True
            print("Pause: Command was successfully paused.")
        queue.delete_command('pause')

    elif queue.check_command('resume'):
        if not pause_status:
            print("Resume: No commands were paused. Skipping command.")
        elif not current_command or not callback_command:
            print("ERROR: Resume: current_command or callback_command is missing. Skipping command.")
        else:
            # TODO: Implement resume command. Resume paused command.
            pause_status = False
            print("Resume: Command was successfully Resumed.")
        queue.delete_command('resume')

    elif queue.check_command('clear'):
        queue.clear_queue()
        print("NOTICE: Queue cleared.") 
    
    elif queue.check_command('skip'):
        if not current_command or not callback_command:
            print("skip: No command is currently executing. Skipping command.")
        else:
            current_command = None
            callback_command = None
            print("Skip: Command was successfully skipped.")
        queue.delete_command('skip')
            
    
    # Check if vehicle is idle. 
    # Existence of a callback_command implies that current_command is being executed right now.
    # (WARNING: callback_command MUST BE RESET TO None when callback_command return "success" or "failure". ALL command functions should have callback. Otherwise, exception should be wroted there. )
    elif not callback_command:
        # Check if current command is picked
        if not current_command:
            # Try to pick a new command (None if queue is empty)
            current_command = queue.pop_queue()
        else:
            # Get function name if command name in alais dictionary
            try:
                print(current_command["command"])
                func_name = next(key for key, value in functions.items() if current_command["command"] in value)
            # Catch exception if command alais name is not in dictionary 
            except StopIteration:
                print("WARNING: Unknown command. Skiped.")
            # Run command
            else:
                # Get arguments
                args = {k: v for k, v in current_command.items() if k != 'command'}
                print(f"args = {args}")
                # Call function and get callback to check command status 
                callback_command = getattr(vehicle, func_name)(**args)
    
    # Checking command status. 
    # Existence of a callback_command implies that current_command is being executed right now.
    else:
        # Calling current command's callback to check command status
        callback_output = callback_command(vehicle)
        
        if callback_output == 'success':
            current_command = None
            callback_command = None
            print("Command finished: " + ', '.join([f'{key}: {value}' for key, value in current_command.items()]))
        
        elif callback_output == 'failure':
            current_command = None
            callback_command = None
            print("WARNING: Command failed. Skiped.")
