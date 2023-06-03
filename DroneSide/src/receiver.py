import socket
import os
import json
import time
from droneQueue import DroneQueue


# Receiver IP address and port
RECEIVER_IP = "127.0.0.1"
RECEIVER_PORT = 9001

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to a specific IP address and port, set timeout for accept
sock.bind((RECEIVER_IP, RECEIVER_PORT))
sock.settimeout(5)

# Initialize the queue
queue = DroneQueue()

# # Directory path to save received JSON files
# RECEIVED_FILES_DIRECTORY = "./received"
# # Create the received files directory if it doesn't exist
# if not os.path.exists(RECEIVED_FILES_DIRECTORY):
#     os.makedirs(RECEIVED_FILES_DIRECTORY)

# Infinite loop for connection
while True:
    try:
        # Listen for incoming connections
        sock.listen(1)
        print("Waiting for sender to connect...")

        # Accept the connection
        connection, sender_address = sock.accept()
        print("Sender connected!")

        while True:
            try:
                # Receive the length of the JSON string
                length_str = connection.recv(1024).decode()
                if not length_str:
                    break
                length = int(length_str)
                # Send acknowledgment
                connection.sendall(b"OK")

                # Receive the JSON string
                json_str = connection.recv(length).decode()
                # Send acknowledgment
                connection.sendall(b"OK")

                # Convert the JSON string to data
                json_data = json.loads(json_str)

                queue.add_to_queue(json_data)

                # Saving json as file
                # # Generate a unique file name for the received JSON file
                # file_name = f"received_{int(time.time())}.json"
                # # Write the JSON data to a file
                # file_path = os.path.join(RECEIVED_FILES_DIRECTORY, file_name)
                # with open(file_path, "w", encoding="utf-8") as file:
                #     json.dump(json_data, file)
                # print(f"Received JSON file: {file_name}")

            except (ValueError, json.JSONDecodeError):
                print("Invalid data received. Skipping...")
            except socket.error:
                print("Connection lost. Retrying...")
                break

        # Close the connection
        connection.close()

    except socket.timeout:
        print("Timeout occurred. Retrying...")

    except socket.error:
        print("Error occurred. Retrying...")
        time.sleep(1)

    except KeyboardInterrupt:
        print("Keyboard interrupt. Closing...")

        try:
            sock.close()
        except socket.error:
            print("Couldn't close socket.")
        break
