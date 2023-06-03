class DroneQueue:
    def __init__(self):
        self.current_command: dict = {}
        self.queue: list = []

    def get_current_command(self):
        return self.current_command

    def get_queue(self):
        return self.queue

    def add_to_queue(self, command):
        self.queue.append(command)

    def is_queue_empty(self):
        return False if self.queue else True

    def next_command(self):
        if self.is_queue_empty():
            return False
        if self.current_command is not None:
            return False
        self.current_command = self.queue.pop(0)
        return True

    def skip_current_command(self):
        self.current_command = {}
