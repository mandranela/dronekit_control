# dronekit_control

python3.9 or less is required.

TODO:

```
Add new commands (disarm)
Implemet pause/resume functionality
logging/output

testing required
special disarm(press), killswitch (hold)
skip: run next command, if none change_mode("HOLD").
```

¿TOFIX?:

```
callback_change_mode requires to wait 0.2 seconds to for mode change check
```

## DroneSide - RPi3

run.py on startup

```
|- Connecting to drone (40 seconds for simulation)
|- Starting reciever.py to get commands from SenderSide
|- Checking for new commands
|- Executing commands
|- Special commands: skip, clear (queue), pause, resume          <--- TODO
|- Each command returns a unique callback to verify its success
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
|- Each command returns a unique callback to verify its success
```

drone_queue.py

```
|- Module responsible for operation with queue files
```

## SenderSide - PC

sender.py

```
|- Takes json files from commands folder
|- Send them to reciever.py
|- If json is empty or broken, moves it to invalid folder
|- If sent successfully, moves it to sent folder
```
