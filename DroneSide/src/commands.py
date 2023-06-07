import argparse
import time
import math
from pymavlink import mavutil
from dronekit import connect, Vehicle, VehicleMode, LocationGlobal, LocationGlobalRelative


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
        print(f"CALLBACK: Mode changed to {value}.")

    def arm_and_takeoff(self, target_alt=2):
        """
        Arms vehicle and fly to target_alt.
        """

        print("Basic pre-arm checks")

        # Don't try to arm until autopilot is ready
        time_counter = 0
        while not self.is_armable:
            print(f"\r Waiting for vehicle to initialise | {time_counter} seconds passed.", end='')
            time_counter += 1
            time.sleep(1)

        print("\nArming motors")
        # Copter should arm in GUIDED mode
        self.mode = VehicleMode("GUIDED")
        self.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.simple_takeoff(target_alt)  # Take off to target altitude

        def callback_arm_and_takeoff(self):
            print(f"\r Altitude: {self.location.global_relative_frame.alt}", end="")
            # Break and return from function just below target altitude.
            if self.location.global_relative_frame.alt:
                if is_location_reached_relative:
                    print("\nSUCCESS: arm_and_takeoff command: Reached target altitude.")
                    return "success"
                else:
                    return "ongoing"
            else:
                print("FAILURE: arm_and_takeoff command: Vehicle do NOT have altitide attribute (location.global_relative_frame.alt).")
                return "failure"

        return callback_arm_and_takeoff


    def fly(self, direction, disctance):
        # Fly. Docs to be added
        original_location = self.location.global_relative_frame

        target_location = original_location

        if direction == "north":
            target_location = get_location_cardinal(original_location, disctance, 0)
        elif direction == "south":
            target_location = get_location_cardinal(original_location, -disctance, 0)
        elif direction == "east":
            target_location = get_location_cardinal(original_location, 0, disctance)
        elif direction == "west":
            target_location = get_location_cardinal(original_location, 0, -disctance)

        elif direction == "forward":
            target_location = get_location_heading(original_location, 0, -disctance)
        elif direction == "backward":
            target_location = get_location_heading(original_location, 0, -disctance)
        elif direction == "right":
            target_location = get_location_heading(original_location, 0, -disctance)
        elif direction == "left":
            target_location = get_location_heading(original_location, 0, -disctance)

        elif direction == "up":
            target_location.alt += disctance
        elif direction == "down":
            target_location.alt -= disctance

        # Check if the vehicle is armed and in GUIDED mode
        if not self.armed:
            print("WARNING: fly command: Drone is not armed!")
        if not self.mode.name == 'GUIDED' or not self.armed:
            print("WARNING: fly command: Drone is not in GUIDED mode!")
        
        # Command the drone to fly to the new location
        self.simple_goto(target_location)

        
        def callback_fly(self):
            if not self.mode.name == "GUIDED":
                print("FAILURE: fly command: Drone is not in GUIDED mode! ")
                return "failure"

            # Considering the rate at which the main loop will run, printing this message does not seem to be a good idea. 
            # remaining_distance = get_distance_metres(self.location.global_relative_frame, target_location)
            # print(f"\rDistance to target: {remaining_distance}")

            if is_location_reached_relative(original_location, self.location.global_relative_frame, target_location):
                print("\nSUCCESS: fly command: Reached target location.")
                return 'success'
            else:
                return 'ongoing'
        
        return callback_fly

    def yaw(self, heading, relative=True):
        
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

        This method sets an absolute heading by default, but you can set the `relative` parameter
        to `True` to set yaw relative to the current yaw heading.

        By default the yaw of the vehicle will follow the direction of travel. After setting 
        the yaw using this function there is no way to return to the default yaw "follow direction 
        of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
        """
        
        original_heading = self.heading
        target_heading = self.heading + heading if relative else heading    # yaw relative to direction of travel
        
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            int(relative),  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.send_mavlink(msg)
        
        def callback_yaw(self):
            if not self.heading:
                print("FAILURE: yaw command: Drone do NOT have heading attribute! ")
                return 'failure'
            elif is_heading_reached_relative(original_heading, self.heading, target_heading):
                print("\nSUCCESS: yaw command: Reached target heading.")
                return 'success'
            else:
                return 'ongoing'
        
        return callback_yaw

    def land(self):
        self.mode = VehicleMode("LAND")
        
        def callback_land(self):
            if self.location.global_relative_frame.alt < 0.1:
                return 'success'
            elif self.mode != VehicleMode("LAND"):
                print(f'FAILURE: land command: Not in land mode above 0.1 meters.')
                return 'failure'
            else:
                return 'ongoing'
        
        return callback_land

    def change_mode(self, mode_name):
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
        
        if mode_name.upper() in [i.upper() for i in modes]:
            self.mode = VehicleMode(mode_name.upper())
        
        def callback_change_mode(self):
            # !!! THERE IS NO WAY TO CHECK IF VEHICLE ACCEPTED NEW MODE OR STILL TRYING TO CHANGE IT.
            # so we will wait 3 seconds and if it no changed then we consider this a failure. goddammit this delay is huge.
            time.sleep(3)
            if self.mode.name.upper() == mode_name.upper():
                return "success"
            else:
                return "failure"
                
        return callback_change_mode
        

def is_heading_reached_relative(original_heading, current_heading, target_heading):
    '''
    This function checks if the current heading of a vehicle is within 5% of the distance between original and target heading.

    :param original_location: Initial position of vehicle.   
    :param current_location: Current position of vehicle.   
    :param target_location: Target location that the vehicle needs to reach.   

    :return bool: True if the vehicle has reached the target location, otherwise it returns False.
    '''
    CLOSE_ENOUGH = 0.05
    
    current_angle = abs(target_heading - current_heading) % 360
    current_angle = min(current_angle, 360 - current_angle)
    
    full_angle = abs(target_heading - original_heading) % 360
    full_angle = min(full_angle, 360 - full_angle)
    
    if current_angle < full_angle * CLOSE_ENOUGH:
        return True
    else:
        return False
    

def is_location_reached_relative(
        original_location: LocationGlobalRelative,
        current_location: LocationGlobalRelative,
        target_location: LocationGlobalRelative
):
    '''
    This function checks if the current location of a vehicle is within 5% of the distance between original and target location.

    :param original_location: Initial position of vehicle.   
    :param current_location: Current position of vehicle.   
    :param target_location: Target location that the vehicle needs to reach.   

    :return bool: True if the vehicle has reached the target location, otherwise it returns False.
    '''
    CLOSE_ENOUGH = 0.05

    if get_distance_metres(original_location, target_location) * CLOSE_ENOUGH > \
            get_distance_metres(current_location, target_location):
        return True
    else:
        return False


def is_location_reached_absolute(
    current_location: LocationGlobalRelative,
    target_location: LocationGlobalRelative
):
    '''
    This function checks if the current location of a vehicle is within 0.2 meters of the target location.

    :param current_location: Current position of vehicle.   
    :param target_location: Target location that the vehicle needs to reach.   

    :return bool: True if the vehicle has reached the target location, otherwise it returns False.
    '''
    CLOSE_ENOUGH = 0.2

    if get_distance_metres(current_location, target_location) < CLOSE_ENOUGH:
        return True
    else:
        return False


def get_distance_metres(
    aLocation1: LocationGlobalRelative,
    aLocation2: LocationGlobalRelative
):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    dalt = aLocation2.alt - aLocation1.alt

    return math.sqrt(math.pow(math.sqrt((dlat*dlat) + (dlong*dlong)) * 111111, 2) + dlat*dalt)


def get_location_heading(original_location: LocationGlobalRelative, distance, heading):
    """
    Move the vehicle forward by a given distance (in meters) based on its current heading.

    :param vehicle: The Vehicle object for the drone.
    :param distance: The distance (in meters) to move the vehicle forward.
    """
    # Calculate the distance in cardinal directions
    dNorth = distance * math.cos(heading)
    dEast = distance * math.sin(heading)

    # Calculate the target location based on the current location, distance, and heading using cardinal directions
    return (get_location_cardinal(original_location, dNorth, dEast))


def get_location_cardinal(original_location: LocationGlobalRelative, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth

    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)

    return LocationGlobal(newlat, newlon, original_location.alt)
