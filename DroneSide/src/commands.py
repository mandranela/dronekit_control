import argparse
import time
import math
from pymavlink import mavutil
from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative
from geopy import distance

def connect_vehicle(connection_string=None):
    """
    Connects to vehicle
    """

    # Set up option parsing to get connection string (OR NOT?)
    # parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
    # parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    # args = parser.parse_args()
    # connection_string = args.connect

    sitl = None
    # Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle
    print(f"Connecting to vehicle on: {connection_string}.")
    vehicle = connect(connection_string, wait_ready=True, vehicle_class=MyVehicle)

    return vehicle


class MyVehicle(Vehicle):
    def __init__(self, *args):
        super(MyVehicle, self).__init__(*args)

        self.add_attribute_listener("mode", self.mode_callback)


    def mode_callback(self, attr_name, value, *args):
        print(f"CALLBACK: Mode changed to: {self.mode.name}.")


    def arm(self):
        '''
        Arms vehicle
        '''
        print("Basic pre-arm checks\n")

        time_start = time.time()

        def callback_arm(self):
            # Don't try to arm until autopilot is ready
            while not self.is_armable:
                print("\033[F", end="") # Move cursor to the beginning of the previous line
                print(f" Waiting for vehicle to become armable... {int(time.time() - time_start)} sec")
                return "ongoing"

            # Copter should arm in GUIDED mode
            self.mode = VehicleMode("GUIDED")
            self.armed = True

            # Confirm vehicle armed before attempting to take off
            if not self.armed:
                print("\033[F", end="") # Move cursor to the beginning of the previous line
                print(" Waiting for arming...")
                return "ongoing" 
            else:
                print("SUCCESS: arm command: Vehicle is armed now")
                return "success"
        
        return callback_arm


    def takeoff(self, target_alt=5):
        """
        Checks if vehicle is armed, in GUIDED mode and fly to target_alt.
        """

        # NOTE: Probably you can delete next line and replace all original_alt in this funciton with 0, since this supposed to be 0 on takeoff. 
        original_alt = self.location.global_relative_frame.alt

        # Stored info whether takeoff msg have been sent
        failed = 0
        if not self.armed:
            print("FAILURE: takeoff command: Drone is not armed!")
            failed = 1
        # Check if vehicle is in GUIDED mode
        elif not self.mode == "GUIDED":
            print(f"FAILURE: takeoff command: Drone is not in GUIDED mode!")
            failed = 1
        # Check if msg to takeoff already were sent (this is neccessary to send back failure msg when )
        else:
            print("Taking off!\n")
            self.simple_takeoff(target_alt)  # Take off to target altitude

        def callback_takeoff(self):
            # failed for some reason didn't saved automatically, so we need to it manually  
            nonlocal failed 
            # Check if vehicle is armed
            if failed:
                return "failure"
            elif not self.armed:
                print("FAILURE: takeoff command: Drone is not armed!")
                return "failure"
            # Check if vehicle is in GUIDED mode
            elif not self.mode == "GUIDED":
                print(f"FAILURE: takeoff command: Drone is not in GUIDED mode!")
                return "failure"
            else:
                # Check if vehicle is reched altitude
                try:
                    if is_altitude_reached_relative(original_alt, self.location.global_relative_frame.alt, target_alt):
                        print("SUCCESS: takeoff command: Reached target altitude.")
                        return "success"
                    else:
                        print("\033[F", end="") # Move cursor to the beginning of the previous line
                        print(f" ONGOING: takeoff command: Altitude: {self.location.global_relative_frame.alt}")
                        return "ongoing"
                except Exception as e:
                    print(f"FAILURE: takeoff command: Exception catched: \n{e}")
                    return "failure"

        return callback_takeoff


    def fly(self, direction="forward", distance=0.1):
        # Fly. Docs to be added
        original_location = self.location.global_relative_frame

        target_location = original_location

        if direction == "north":        target_location = get_location_cardinal(original_location,  distance, 0)
        elif direction == "south":      target_location = get_location_cardinal(original_location, -distance, 0)
        elif direction == "east":       target_location = get_location_cardinal(original_location, 0,  distance)
        elif direction == "west":       target_location = get_location_cardinal(original_location, 0, -distance)

        elif direction == "forward":    target_location = get_location_heading(original_location, distance,   0)
        elif direction == "backward":   target_location = get_location_heading(original_location, distance, 180)
        elif direction == "right":      target_location = get_location_heading(original_location, distance,  90)
        elif direction == "left":       target_location = get_location_heading(original_location, distance, 270)

        elif direction == "up":         target_location.alt += distance
        elif direction == "down":       target_location.alt -= distance

        # Check if the vehicle is armed and in GUIDED mode
        if not self.armed:
            print("WARNING: fly command: Drone is not armed!")
        if not self.mode.name == 'GUIDED' or not self.armed:
            print("WARNING: fly command: Drone is not in GUIDED mode!")
        
        # Command the drone to fly to the new location
        self.simple_goto(target_location)
        
        print(f" ONGIONG: fly command: Flying to target: {(target_location.lat, target_location.lon, target_location.alt)}\n")
        def callback_fly(self):
            try:
                if not self.mode.name == "GUIDED":
                    print("FAILURE: fly command: Drone is not in GUIDED mode! ")
                    return "failure"
                if not self.armed:
                    print("FAILURE: fly command: Drone is not armed! ")
                    return "failure"
                
                # Considering the rate at which the main loop will run, printing this message does not seem to be a good idea. 

                if is_location_reached_absolute(self.location.global_relative_frame, target_location):
                    print("SUCCESS: fly command: Reached target location.")
                    return 'success'
                else:
                    print("\033[F", end="") # Move cursor to the beginning of the previous line
                    print(f" ONGIONG: fly command: Distance to target: {get_distance_metres(self.location.global_relative_frame, target_location)}")
                    return 'ongoing'
            except Exception as e:
                print(f"FAILURE: fly command: Exception catched: \n{e}")
                return "failure"
            
        return callback_fly


    def yaw(self, heading=180, relative=True):        
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` or 1 to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting 
        the yaw using this function there is no way to return to the default yaw "follow direction 
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
        """
        
        original_heading = self.heading # Used for is_heading_reached_relative
        target_heading = (self.heading + heading) % 360 if relative else heading % 360   # yaw relative to direction of travel
        
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading % 360,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            int(relative),  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.send_mavlink(msg)
        
        print(f" ONGOING: yaw command: Changing heading to {target_heading} (where north = 0)\n")
        def callback_yaw(self):
            try:
                if is_heading_reached_absolute(self.heading, target_heading):
                    print("SUCCESS: yaw command: Reached target heading.")
                    return 'success'
                else:
                    print("\033[F", end="") # Move cursor to the beginning of the previous line
                    print(f" ONGIONG: yaw command: Angle to target: {get_angle_distance(self.heading, target_heading)}")
                    return 'ongoing'
            except Exception as e:
                print(f"FAILURE: yaw command: Exception catched: \n{e}")
                return "failure"
        
        return callback_yaw


    def land(self):
        self.mode = VehicleMode("LAND")
        time.sleep(0.5)

        print(f" ONGIONG: land command: Changing mode to land")
        def callback_land(self):
            try:
                if self.location.global_relative_frame.alt < 0.1:
                    return 'success'
                elif self.mode != VehicleMode("LAND"):
                    print(f'FAILURE: land command: Not in land mode above 0.1 meters.')
                    return 'failure'
                else:
                    print("\033[F", end="") # Move cursor to the beginning of the previous line
                    print(f" ONGIONG: land command: Altitude above home: {self.location.global_relative_frame.alt}")
                    return 'ongoing'
            except Exception as e:
                print(f"FAILURE: land command: Exception catched: \n{e}")
                return "failure"
        
        return callback_land


    def RTL(self):
        # RTL (assumed to land)
        self.mode = VehicleMode("RTL")
        time.sleep(0.2)
        
        def callback_RTL(self):
            try:
                if self.mode.name != "RTL":
                    return "failure"
                elif is_location_reached_absolute(self.location.global_frame, self.home_location):
                    return "success"
                else:
                    QUEUE_PATH = './src/queue.txt'
                    print(f" ONGOING: RTL command: Distance to home: {get_distance_metres(self.location.global_frame, self.home_location)}")
                    return "ongoing"
            except Exception as e:
                print(f"FAILURE: land command: Exception catched: \n{e}")
                return "failure"

        return callback_RTL
    

    def change_mode(self, mode):
        modes = [
                    "Acro",
                    "Altitude Hold",
                    "AirMode",
                    "Auto",
                    "Brake",
                    "Circle",
                    "Drift",
                    "Flip",
                    "FlowHold",
                    "Follow",
                    "Follow Me",
                    "Guided",
                    "Heli_Autorotate",
                    "Land",
                    "Loiter",
                    "PosHold",
                    "RTL",
                    "Simple and Super Simple",
                    "Smart RTL",
                    "Sport",
                    "Stabilize",
                    "System Identification",
                    "Throw",
                    "Turtle",
                    "ZigZag"
                ]
        
        if mode.upper() in [i.upper() for i in modes]:
            self.mode = VehicleMode(mode.upper())
        
        def callback_change_mode(self):
            # NOTE: There is should be better way to check if vehicle accepted/declined or still trying to process mode change.
            # But I don't find one, so for now we will wait 0.2 seconds and if it didn't change then we consider this as a failure.
            try:
                time.sleep(0.1)
                if self.mode.name.upper() == mode.upper():
                    return "success"
                else:
                    return "failure"
            except Exception as e:
                print(f"FAILURE: change_mode command: Exception catched: \n{e}")
                return "failure"
                
        return callback_change_mode




### AUXILIARY FUNCTIONS ###


### CHECK TARGET REACHING ###

# NOTE: 5% seems to be too much. TBH count this relatively is kinda stoopid. Use absolute instead
def is_heading_reached_relative(original_heading, current_heading, target_heading, PRECISION_RELATIVE = 0.05):
    '''
    This function checks if the current heading of a vehicle is within 5% of the angle between original and target heading.

    :param original_location: Initial position of vehicle.   
    :param current_location: Current position of vehicle.   
    :param target_location: Target location that the vehicle needs to reach.   

    :return bool: True if the vehicle has reached the target location, otherwise it returns False.
    '''    
    current_angle = get_angle_distance(target_heading, current_heading)
    full_angle = get_angle_distance(target_heading, original_heading)
    
    return True if current_angle < full_angle * PRECISION_RELATIVE else False


def is_heading_reached_absolute(current_heading, target_heading, PRECISION_ABSOLUTE = 1):
    '''
    This function checks if the current heading of a vehicle is within 1 degree of the angle between original and target heading.

    :param original_location: Initial position of vehicle.   
    :param current_location: Current position of vehicle.   
    :param target_location: Target location that the vehicle needs to reach.   

    :return bool: True if the vehicle has reached the target location, otherwise it returns False.
    '''        
    return True if get_angle_distance(target_heading, current_heading) < PRECISION_ABSOLUTE else False


def is_altitude_reached_relative(original_alt, current_alt, target_alt, PRECISION_RELATIVE = 0.05):    
    return True if abs(current_alt - target_alt) < abs(target_alt - original_alt) * PRECISION_RELATIVE else False


def is_altitude_reached_absolute(original_alt, current_alt, target_alt, PRECISION_ABSOLUTE = 1):    
    return True if abs(current_alt - target_alt) < abs(target_alt - original_alt) * PRECISION_ABSOLUTE else False


def is_location_reached_relative(
    original_location: LocationGlobalRelative, 
    current_location:  LocationGlobalRelative, 
    target_location:   LocationGlobalRelative,
    PRECISION_RELATIVE = 0.05
):
    '''
    This function checks if the current location of a vehicle is within 5% of the distance between original and target location.

    :param original_location: Initial position of vehicle.   
    :param current_location: Current position of vehicle.   
    :param target_location: Target location that the vehicle needs to reach.   

    :return bool: True if the vehicle has reached the target location, otherwise it returns False.
    '''

    if  get_distance_metres(current_location, target_location) < \
        get_distance_metres(original_location, target_location) * PRECISION_RELATIVE:
        return True
    else:
        return False


def is_location_reached_absolute(current_location: LocationGlobalRelative, target_location: LocationGlobalRelative, PRECISION_ABSOLUTE = 1):
    '''
    This function checks if the current location of a vehicle is within 1 meters of the target location.

    :param current_location: Current position of vehicle.   
    :param target_location: Target location that the vehicle needs to reach.   

    :return bool: True if the vehicle has reached the target location, otherwise it returns False.
    '''

    if get_distance_metres(current_location, target_location) < PRECISION_ABSOLUTE:
        return True
    else:
        return False


### FUNCITONS TO GET DIFFERENCES ###


def get_angle_distance(angle1, angle2):
    tempAngle = abs(angle1 - angle2) % 360
    return min(tempAngle, 360 - tempAngle)


def get_ground_distance_metres(aLocation1: LocationGlobalRelative, aLocation2: LocationGlobalRelative):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    return distance.distance((aLocation1.lat, aLocation1.lon), (aLocation2.lat, aLocation2.lon)).m


def get_distance_metres(aLocation1: LocationGlobalRelative, aLocation2: LocationGlobalRelative):
    """
    Returns the distance in metres between two LocationGlobal objects.
    """
    return math.sqrt(distance.distance((aLocation1.lat, aLocation1.lon), (aLocation2.lat, aLocation2.lon)).m + math.pow(aLocation2.alt - aLocation1.alt, 2))


def get_location_heading(original_location: LocationGlobalRelative, distance, heading):
    """
    Returns a LocationGlobal object containing the latitude/longitude `distance` metres 
    towards `heading` angle (where 0 is north) from the specified `original_location`. 
    The returned Location has the same `alt` value as `original_location`.
    """
    # Calculate the distance in cardinal directions
    dNorth = distance * math.cos(heading)
    dEast = distance * math.sin(heading)

    # Calculate the target location based on the current location, distance, and heading using cardinal directions
    return get_location_cardinal(original_location, dNorth, dEast)


def get_location_cardinal(original_location: LocationGlobalRelative, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    """
    # Offset in meters
    offset = distance.distance(meters=dNorth).destination((original_location.lat, original_location.lon), 0)
    offset_lat, offset_lon = distance.distance(meters=dEast).destination(offset, 90)[:2]

    return LocationGlobalRelative(offset_lat, offset_lon, original_location.alt)