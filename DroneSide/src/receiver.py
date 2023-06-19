import os
import socket
import json
import time
import logging

import drone_queue


### PUBLIC PROPERTIES ###

# Receiver IP address and port
RECEIVER_IP = "127.0.0.1"
RECEIVER_PORT = 9001

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


def connect_to_drone(sock):
    while True:
        try:
            # Listen for incoming connections
            sock.listen(1)
            # Accept the connection
            connection, sender_address = sock.accept()
            logging.info("Sender connected!")
            return connection
        
        except socket.timeout:
            logging.warning("Timeout occurred. Retrying...")
            continue

        except socket.error:
            logging.warning("Socket error occurred. Retrying...")
            continue
        

def get_json(connection):
    try:
        # Receive the length of the JSON string
        length_str = connection.recv(1024).decode()
        if not length_str:
            return False
        length = int(length_str)
        
        # Send acknowledgment
        connection.sendall(b"OK")

        # Receive the JSON string
        json_str = connection.recv(length).decode()
        # Send acknowledgment
        connection.sendall(b"OK")

        # Convert the JSON string to data
        json_data = json.loads(json_str)

        # Check command and append in corresponding queue
        if "command" in json_data:
            if any(json_data["command"] in value_list for value_list in specials.values()):
                drone_queue.append_to_queue(json_data, special=True)
                logging.info(f"Received special command: " + ', '.join([f'{key}: {value}' for key, value in json_data.items()]))
            elif any(json_data["command"] in value_list for value_list in functions.values()):
                drone_queue.append_to_queue(json_data)
                logging.info(f"Received regular command: " + ', '.join([f'{key}: {value}' for key, value in json_data.items()]))
            else:
                logging.warning(f"Received invalid command: " + ', '.join([f'{key}: {value}' for key, value in json_data.items()]))
        
        return True

    except (ValueError, json.JSONDecodeError) as e:
        logging.warning("Invalid data received. Skipping...")
        return e
        
    except socket.error as e:
        logging.warning("Connection lost. Retrying...")
        # Close the connection
        connection.close()
        return e
    

### MAIN FUNCTION ###
def receive():
    # Configurate logging 
    logging.basicConfig(filename='./logs/receiver.log', level=logging.DEBUG)
    # Create a socket object
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Bind the socket to a specific IP address and port, set timeout for accept
    sock.bind((RECEIVER_IP, RECEIVER_PORT))
    sock.settimeout(5)
    
    logging.info("Waiting for sender to connect...")
    connection = connect_to_drone(sock)
    while True:
        result = get_json(connection)
        if result == socket.error:
            connection = connect_to_drone(sock)


if __name__ == "__main__":
    try:
        receive()
    except KeyboardInterrupt:
        logging.warning("INTERRUPTION: Keyboard interrupt. Closing...")
        exit()
