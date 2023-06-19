import socket
import os
import json
import time
import shutil
import time
import logging

### PUBLIC PROPERTIES ###

# Receiver IP address and port
receiver_ip = "127.0.0.1"
receiver_port = 9001

# Directory path for JSON files to send
json_files_directory = "./commands"
waiting_directory = json_files_directory + "/waiting"
sent_directory = json_files_directory + "/sent"
invalid_directory = json_files_directory + "/invalid"


### FUNCTIONS ###

def connect_to_drone(sock):
    # Connect to the receiver
    while True:
        try:
            sock.connect((receiver_ip, receiver_port))
            logging.info("Connected to receiver")
            break
        except socket.error:
            logging.warning("Connection failed. Retrying...")
            time.sleep(0.1)


def move_file(source_path, destination_folder):
    try:
        filename = os.path.basename(source_path)
        destination_path = os.path.join(destination_folder, filename)
        shutil.move(source_path, destination_path)
        logging.info(f"File moved from {filename} to {destination_folder}")
    except FileNotFoundError:
        logging.warning(f"Source file {source_path} not found")
    except PermissionError:
        logging.warning(f"Permission denied. Unable to move file")
    except Exception as e:
        logging.warning(f"An error occurred while moving the file: {str(e)}")


def send_json(sock, file_path):
    # Send JSON file to the receiver
    try:
        # Read the JSON file
        with open(file_path, "r") as file:
            json_data = json.load(file)

        # Convert JSON data to string
        json_str = json.dumps(json_data)

        while True:
            try:
                # Send the length of the JSON string
                sock.sendall(str(len(json_str)).encode())

                # Wait for acknowledgment
                ack = sock.recv(2)
                if ack != b"OK":
                    logging.warning(f"Unable to send length of JSON file: {file_path}. Retrying...")
                    time.sleep(1)
                    continue
                
                # Send the JSON string
                sock.sendall(json_str.encode())

                # Wait for acknowledgment
                ack = sock.recv(2)
                if ack == b"OK":
                    logging.info(f"JSON file {file_path} sent successfully")
                    move_file(file_path, sent_directory)
                    return True
                else:
                    logging.warning(f"Unable to send JSON file: {file_path}. Retrying...")
            except socket.error as e:
                logging.warning("Connection lost. Retrying..")
                return e
            time.slee(0.1)

    except (ValueError, json.JSONDecodeError) as e:
        logging.warning(f"Invalid JSON file: {file_path}. Skipping..")
        move_file(file_path, invalid_directory)
        return e


if __name__ == "__main__":
    # Setting logging
    logging.basicConfig(filename='sender.log', level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    logging.info("sender.py activated")
    
    # Create a socket object and connect it to the drone
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connect_to_drone(sock)
    # Continuously send the oldest JSON file in the directory
    while True:
        # Check for JSON files in the directory
        json_files = [f for f in os.listdir(waiting_directory) if f.endswith(".json")]
        if not json_files:
            logging.warning("No JSON files to send. Waiting... ")
            while not json_files:
                json_files = [f for f in os.listdir(waiting_directory) if f.endswith(".json")]
                time.sleep(0.01)

        # Find the oldest JSON file
        oldest_file = min(json_files, key=lambda f: os.path.getctime(os.path.join(waiting_directory, f)))
        oldest_file_path = os.path.join(waiting_directory, oldest_file)

        # Attempt to send the oldest file
        result = send_json(sock, oldest_file_path)
        
        if result is socket.error:
            connect_to_drone(sock)
            
