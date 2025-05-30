import config
import time


class RobotCommander:
    def __init__(self, mqtt_handler):
        self.mqtt_handler = mqtt_handler
        self.last_command_time = 0
        self.command_interval = 0.05

    def _can_send_command(self):
        can_send = time.time() - self.last_command_time > self.command_interval
        if can_send:
            self.last_command_time = time.time()
        return can_send

    def send_command(self, command):
        if self._can_send_command():
            print(f"Sending command to robot: {command}")
            self.mqtt_handler.publish(config.TOPIC_ROBOT_GPIO, command)

    def forward(self):
        self.send_command("forward")

    def backward(self):
        self.send_command("backward")

    def left(self):
        self.send_command("left")

    def right(self):
        self.send_command("right")

    def stop(self):
        self.send_command("stop")

    def shutdown_client(self):
        self.send_command("shutdown_client")
