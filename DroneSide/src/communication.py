import os
import json


RECEIVED_DIRECTORY = "./received"


def get_new_commands():
    # Check for JSON files in the directory
    json_files = [f for f in os.listdir(RECEIVED_DIRECTORY) if f.endswith(".json")]

    if not json_files:
        return None

    # Find the oldest JSON file
    oldest_file = min(json_files, key=lambda f: os.path.getctime(os.path.join(RECEIVED_DIRECTORY, f)))
    oldest_file_path = os.path.join(RECEIVED_DIRECTORY, oldest_file)

    with open(oldest_file_path) as json_file:
        data = json.load(json_file)
    return data
