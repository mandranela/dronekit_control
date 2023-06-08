'''
This module responsible for operation with queue.txt file.
Supposed to automatically create queue.txt file and handle errors, that might occur when run.py and receiver.py will compete for it.

Available commands: 
get_queue()       -> list[dict] returns current list of commands in queue
append_to_queue() -> list[dict] returns updated list of commands in queue
pop_queue()       -> dict       returns and delete first command in queue
is_empty()        -> bool       returns boolean indicating if queue is empty
'''

import time
import json 

QUEUE_PATH = './src/queue.txt'


def queue_operation(operation):
    def wrapper(*args, path=QUEUE_PATH):
        # Try to open the queue file for writing, retrying if necessary
        while True:
            try:
                with open(path, 'r+') as f:
                     # Check if the file has any content
                    f.seek(0)
                    content = f.read()
                    if not content.strip():
                        # If the file is empty, write an empty list to it
                        f.seek(0)
                        f.write('[]\n')
                        f.truncate()
                    f.seek(0)
                        
                    # Perform the operation on the file
                    return(operation(f, *args))
                
            except FileNotFoundError:
                # If the file doesn't exist, create it with empty list and retry
                with open(path, 'w') as f:
                    f.write('[]\n')

            except PermissionError:
                # If the file is locked, wait for a short time and retry
                print("File is locked. Retrying in 0.2 second...")
                time.sleep(0.2)
    
    return wrapper


@queue_operation
def get_queue(f) -> list[dict]:
    # Return a queue
    return json.load(f)


@queue_operation
def pop_queue(f) -> dict:
    # Return first command from queue
    queue = json.load(f)
    
    # Pop first command if exist, else None
    item = queue.pop(0) if queue else None
    
    # Write updated queue back to the file
    f.seek(0)
    f.truncate()
    json.dump(queue, f, indent=4)
    
    # Return first command
    return item


@queue_operation
def append_to_queue(f, json_data: dict) -> list[dict]: 
    # Append new command in the end of queue. json_data - dict (json.loads(str))
    queue = json.load(f)

    # Append the new item to the queue and write it back to the file
    queue.append(json_data)
    
    # Write updated queue back to the file
    f.seek(0)
    f.truncate()
    json.dump(queue, f, indent=4)
    
    # Return updated queue
    return queue


@queue_operation
def clear_queue(f):
    # Clear the queue
    try:
        f.seek(0)
        f.truncate()
        f.write('[]\n')
        return True
    
    except Exception:
        return False

@queue_operation
def delete_command(f, command):
    # Delete all certain commands from the queue
    queue = json.load(f)
    queue = [item for item in queue if not (item.get("command") == command)]
    
    # Write updated queue back to the file
    f.seek(0)
    f.truncate()
    json.dump(queue, f, indent=4)
    
    # Return updated queue
    return queue


@queue_operation
def check_command(f, command):
    # Check if a certain commands in the queue
    queue = json.load(f)
    
    # Check for commands
    if any(item.get("command") == command for item in queue):
        return True
    return False


    
@queue_operation
def is_empty(f) -> bool:
    queue = json.load(f)
    return not bool(queue)


if __name__ == "__main__":
    # Test queue operations
    
    json_str = '{ "command": "fly", "arg1": 5, "arg2": "left", "etc": "etc" }'
    json_data = json.loads(json_str)

    print(json.dumps(get_queue()))
    print(json.dumps(append_to_queue(json_data)))
    print(json.dumps(get_queue()))
    print(pop_queue())
    print(json.dumps(get_queue()))
    print(is_empty())

