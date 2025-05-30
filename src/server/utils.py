import time

class RateLimiter:
    def __init__(self, rate_hz):
        if rate_hz <= 0:
            raise ValueError("Rate must be positive.")
        self.period = 1.0 / rate_hz
        self.last_time = 0

    def sleep(self):
        current_time = time.monotonic()
        elapsed = current_time - self.last_time
        sleep_duration = self.period - elapsed
        if sleep_duration > 0:
            time.sleep(sleep_duration)
        self.last_time = time.monotonic()

def print_system_info():
    print("-" * 30)
    print("Robot Control Server")
    print(f"MQTT Broker: {config.MQTT_BROKER_IP}:{config.MQTT_PORT}")
    print(f"Listening for commands on: {config.TOPIC_LAPTOP_COMMANDS}")
    print(f"Sending robot commands to: {config.TOPIC_ROBOT_GPIO}")
    print("-" * 30)
    print("Controls:")
    print("  MQTT Commands (to topic laptop/commands):")
    print("    'start_vision'  - Enable autonomous mode")
    print("    'stop_vision'   - Disable autonomous mode (manual control)")
    print("    'toggle_camera' - Toggle camera display window")
    print("    'shutdown'      - Stop this server application")
    print("    'forward', 'backward', 'left', 'right', 'stop' - Manual robot control")
    print("  Keyboard Shortcuts (when OpenCV window is active):")
    print("    'q' - Quit application")
    print("    'v' - Toggle vision (autonomous) mode")
    print("    'c' - Toggle camera display")
    print("    'w','a','s','d','x' - Manual robot movement (w-fwd,s-stop,a-left,d-right,x-bwd)")
    print("-" * 30)

import config
