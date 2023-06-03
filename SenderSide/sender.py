import socket
import os
import json
import time
import shutil
import time


class Print_sc:
    def __init__(self):
        self.counter = 0

    def print_sc(self, msg):
        self.counter += 1
        formatted_msg = f"\033[F{msg} ({self.counter} seconds)"
        print(formatted_msg)
        time.sleep(1)


# Receiver IP address and port
receiver_ip = "127.0.0.1"
receiver_port = 9001

# Directory path for JSON files to send
json_files_directory = "./commands"
waiting_directory = json_files_directory + "/waiting"
sent_directory = json_files_directory + "/sent"
invalid_directory = json_files_directory + "/invalid"


# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the receiver
while True:
    try:
        sock.connect((receiver_ip, receiver_port))
        print("Connected to receiver.")
        break
    except socket.error:
        print("Connection failed. Retrying...")
        time.sleep(1)


def move_file(source_path, destination_folder):
    try:
        filename = os.path.basename(source_path)
        destination_path = os.path.join(destination_folder, filename)
        shutil.move(source_path, destination_path)
        print(f"File moved from {filename} to {destination_folder} successfully.\n")
    except FileNotFoundError:
        print(f"Source file {source_path} not found.")
    except PermissionError:
        print(f"Permission denied. Unable to move file.")
    except Exception as e:
        print(f"An error occurred while moving the file: {str(e)}")


def send_json(file_path):
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
                    print(f"Failed to send JSON file {file_path}. Retrying...")
                    continue

                # Send the JSON string
                sock.sendall(json_str.encode())

                # Wait for acknowledgment
                ack = sock.recv(2)
                if ack == b"OK":
                    print(f"JSON file {file_path} sent successfully.")
                    move_file(file_path, sent_directory)
                    break
                else:
                    print(f"Failed to send JSON file {file_path}. Retrying...")
            except socket.error:
                print("Connection lost. Retrying...")
                break

    except (ValueError, json.JSONDecodeError):
        print(f"Invalid JSON file: {file_path}. Skipping...")
        move_file(file_path, invalid_directory)


# Continuously send the oldest JSON file in the directory
Printer = Print_sc()
while True:
    # Check for JSON files in the directory
    json_files = [f for f in os.listdir(waiting_directory) if f.endswith(".json")]

    if not json_files:
        Printer.print_sc("No JSON files to send. Waiting... ")
        continue
    Printer.counter = 0

    # Find the oldest JSON file
    oldest_file = min(json_files, key=lambda f: os.path.getctime(os.path.join(waiting_directory, f)))
    oldest_file_path = os.path.join(waiting_directory, oldest_file)

    # Attempt to send the oldest file
    send_json(oldest_file_path)
