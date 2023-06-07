# dronekit_control

didn't tested this one with sitl. might be a lot of bugs, but i'm tired.

### DroneSide - RPi3 

TODO:
''
Add new commands
Implemet pause/resume functionality
''

TOFIX?:
''
arm_and_takeoff: it waits for arm to succeed which delay main loop until it'll finishes
callback_change_mode requires to wait 3 seconds to for mode change check
''

run.py on startup
```
|- Connecting to drone (40 seconds for simulation)
|- Starting reciever.py to get commands from SenderSide
|- Checking for new commands 
|- Executing commands 
|- Special commands: skip, clear (queue), pause, resume          <--- TODO
|- Each command returns a unique callback to verify its success  <--- TODO
```

reciever.py
```
|- Constantly trying to connect to sender and get new commands
|- Sends new commands to queue
```

commands.py
```
|- Functions for actions, checking distances, telemetry, etc.
|- myVehicle extended Vehicle class with commands for actions. Probably should separate from non-class functions  
|- Each command returns a unique callback to verify its success  <--- TODO
```

droneQueue.py
```
|- Simple class for commands queue. Perhaps will move it to myVehicle class in commands.py
```

### SenderSide - PC                             

sender.py
```
|- Takes json files from commands folder
|- Send them to reciever.py
|- If json is empty or broken, moves it to invalid folder
|- If sent successfully, moves it to sent folder
```
