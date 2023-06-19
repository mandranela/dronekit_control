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

os.chdir = ("~/dronekit_control/Droneside/")

from src import receiver
from src import drone_queue
from src.commands import MyVehicle, connect_vehicle



CONNECTION_STRING = None
# Uncomment next line for RPi3 real pixhawk connection
# CONNECTION_STRING = "tcp:127.0.0.1:911"

# UPDATE ALIAS IN reciever.py when modifing it there.
# Alais for commands. Keys are declared function's names and values are recieved command values from sender. Exmaple: {"command": "fly"}
functions = {
    "arm": ["arm", "ARM"],
    "takeoff": ["takeoff", "TAKEOFF"],
    "fly": ["fly", "move", "FLY", "MOVE"],
    "yaw": ["yaw", "rotate", 'YAW', "ROTATE"],
    "RTL": ["rtl", "RTL"],
    "land": ["land", "LAND"],
    "change_mode": ["mode", "change mode", "change_mode", "mode change", "mode_change", "MODE", "CHANGE_MODE"],
}
# Alais for special commands.
specials = {
    "pause": ["pause"],
    "resume": ["resume"],
    "clear": ["clear"],
    "skip": ["skip"]
}
# Initialize current command and special variables
current_command = None
callback_command = None

special_command = None
pause_status = False



# Start the receiver.py in a separate thread
receiver_thread = threading.Thread(target=receiver.receive)
receiver_thread.start()

# Connect to vehicle on startup
vehicle = connect_vehicle(connection_string=CONNECTION_STRING)

# Main loop
while True:
    # Check for special commands
    if not drone_queue.is_empty(special=True):
        special_command_name = drone_queue.pop_queue(special=True)["command"]
        try:
            special_command = next(key for key, value in specials.items() if special_command_name in value)
        except StopIteration:
            print(f"WARNING: Unknown SPECIAL command: {special_command_name}. Skiped.")

        print(f"SPECIAL command got: {special_command}")
        if special_command == 'pause':
            if not current_command or not callback_command:
                print("Pause: No command is currently executing. Skipping command.")
            else:
                # TODO: Implement pause command. Pause current command with capability to comtinue it in future with resume command.
                pause_status = True
                print("Pause: Command was successfully paused.")

        elif special_command == 'resume':
            if not pause_status:
                print("Resume: No commands were paused. Skipping command.")
            elif not current_command or not callback_command:
                print("ERROR: Resume: current_command or callback_command is missing. Skipping command.")
            else:
                # TODO: Implement resume command. Resume paused command.
                pause_status = False
                print("Resume: Command was successfully Resumed.")

        elif special_command == 'clear':
            drone_queue.clear_queue(special=True)
            print("NOTICE: Spcial queue cleared.") 
        
        elif special_command == 'skip':
            if not current_command or not callback_command:
                print("SKIP: No command is currently executing.")
            else:
                current_command = None
                callback_command = None
                print("SKIP: Command was successfully skipped.")

        special_command = None
            
    
    # Check for new commands to execute
    # Existence of a callback_command implies that current_command is being executed right now. (WARNING: callback_command MUST BE RESET TO None when callback_command return "success" or "failure". ALL command functions should have callback. Otherwise, exception should be wroted there. )
    elif not callback_command:
        # Check if current command is picked
        if not current_command:
            # Try to pick a new command (None if queue is empty)
            current_command = drone_queue.pop_queue()
        else:
            # Get function name if command name in alais dictionary
            try:
                func_name = next(key for key, value in functions.items() if current_command['command'] in value)
                print(f"\nINFO: Executing new commmand: {current_command['command']}")
            # Catch exception if command alais name is not in dictionary 
            except StopIteration:
                print("WARNING: Unknown command. Skiped.")
                current_command = None
            # Run command
            else:
                # Get arguments
                args = {k: v for k, v in current_command.items() if k != 'command'}
                print(f" Args: {args}")
                # Call function and get callback to check command status 
                callback_command = getattr(vehicle, func_name)(**args)
    
    # Checking command status. 
    else:
        # Calling current command's callback to check command status
        callback_output = callback_command(vehicle)
        
        if callback_output == 'success':
            print("Command finished: " + ', '.join([f'{key}: {value}' for key, value in current_command.items()]))
            current_command = None
            callback_command = None
        
        elif callback_output == 'failure':
            print("WARNING: Command failed. Skiped.")
            current_command = None
            callback_command = None
