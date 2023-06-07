import os
import socket
import json
import time

import queue

# Receiver IP address and port
RECEIVER_IP = "127.0.0.1"
RECEIVER_PORT = 9001

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to a specific IP address and port, set timeout for accept
sock.bind((RECEIVER_IP, RECEIVER_PORT))
sock.settimeout(5)


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

                queue.append_to_queue(json_data)
                print(f"Received command: " + ', '.join([f'{key}: {value}' for key, value in json_data.items()]))

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
