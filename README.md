# dronekit_control

### DroneSide - RPi3 

run.py on startup
``
|- Connecting to drone (40 seconds for simulation) 
|- Starting reciever.py to get commands from SenderSide 
|- Checking for new commands 
|- Executing commands 
|- Each command returns a unique callback to verify its success  <--- TODO
|- Check for stop signal                                         <--- TODO
``

reciever.py
``
|- Constantly trying to connect to sender and get new commands
|- Send new commands to queue                                    <--- TOFIX (implemented this in main loop, now I want it being independent for stop signal)      
``

commands.py
``
|- Functions for actions, checking distances, telemetry, etc.
|- myVehicle extended Vehicle class with commands for actions. Probably should separate from non-class functions  
|- Each command returns a unique callback to verify its success  <--- TODO
``

droneQueue.py
``
|- Simple class for commands queue. Perhaps will move it to myVehicle class in commands.py
``

### SenderSide - PC                             

sender.py
``
|- Takes json files from commands folder
|- Send them to reciever.py
|- If json is empty or broken, moves it to invalid folder
|- If sent successfully, moves it to sent folder
``
