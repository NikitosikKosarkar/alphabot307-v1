import time
import signal
import sys
import numpy as np

import config
from mqtt_client_handler import MQTTClientHandler
from camera_manager import CameraManager
from vision_processor import VisionProcessor
from robot_commander import RobotCommander
from utils import RateLimiter, print_system_info


class MainServer:
    def __init__(self):
        self.running = False
        self.autonomous_mode = False

        self.mqtt_handler = None
        self.camera_manager = None
        self.vision_processor = None
        self.robot_commander = None

        self.last_autonomous_action_time = 0
        self.target_lost_search_active = False
        self.target_lost_search_start_time = 0

    def initialize(self):
        try:
            self.mqtt_handler = MQTTClientHandler(
                config.MQTT_BROKER_IP, config.MQTT_PORT, config.MQTT_CLIENT_ID
            )
            self.mqtt_handler.set_message_callback(self._handle_mqtt_message)
            if not self.mqtt_handler.connect():
                print("Failed to connect to MQTT. Exiting.")
                return False
            self.mqtt_handler.start_loop()

            self.camera_manager = CameraManager()
            self.vision_processor = VisionProcessor()
            self.robot_commander = RobotCommander(self.mqtt_handler)

            self.running = True
            print_system_info()
            return True
        except Exception as e:
            print(f"Error during initialization: {e}")
            self.cleanup()
            return False

    def _handle_mqtt_message(self, topic, payload):
        payload = payload.lower().strip()
        print(f"Server received MQTT on '{topic}': {payload}")

        if topic == config.TOPIC_LAPTOP_COMMANDS:
            if payload == "start_vision":
                self.autonomous_mode = True
                self.target_lost_search_active = False
                print("Autonomous vision mode ACTIVATED.")
            elif payload == "stop_vision":
                self.autonomous_mode = False
                self.robot_commander.stop()
                print("Autonomous vision mode DEACTIVATED. Manual control enabled.")
            elif payload == "toggle_camera":
                if self.camera_manager:
                    self.camera_manager.toggle_display()
            elif payload == "shutdown":
                print("Shutdown command received via MQTT.")
                self.running = False
            elif payload in ["forward", "backward", "left", "right", "stop"]:
                if not self.autonomous_mode:
                    getattr(self.robot_commander, payload)()
                else:
                    print(f"Manual command '{payload}' ignored in autonomous mode.")
            else:
                print(f"Unknown command on {config.TOPIC_LAPTOP_COMMANDS}: {payload}")
        elif topic == config.TOPIC_ROBOT_STATUS:
            print(f"Status from Robot: {payload}")

    def _handle_keyboard_input(self, key_event):
        if key_event == 'quit':
            self.running = False
        elif key_event == 'toggle_vision':
            self.autonomous_mode = not self.autonomous_mode
            self.target_lost_search_active = False
            if not self.autonomous_mode:
                self.robot_commander.stop()
            print(f"Autonomous mode {'ENABLED' if self.autonomous_mode else 'DISABLED'} by keyboard.")
        elif key_event == 'toggle_camera':
            if self.camera_manager:
                self.camera_manager.toggle_display()
        elif key_event in ['forward', 'backward', 'left', 'right', 'stop']:
            if not self.autonomous_mode:
                if key_event == 's':
                    self.robot_commander.stop()
                elif key_event == 'w':
                    self.robot_commander.forward()
                elif key_event == 'a':
                    self.robot_commander.left()
                elif key_event == 'd':
                    self.robot_commander.right()
                elif key_event == 'x':
                    self.robot_commander.backward()

            else:
                print(f"Manual key '{key_event}' ignored in autonomous mode.")

    def _run_autonomous_control(self):
        if not self.autonomous_mode or not self.vision_processor:
            return

        angle = self.vision_processor.get_angle_to_target()
        distance = self.vision_processor.get_distance_to_target()

        current_time = time.time()

        if current_time - self.last_autonomous_action_time < config.TURN_COMMAND_DURATION_S:
            return

        if self.vision_processor.target_pos[0] < 0:
            if not self.target_lost_search_active:
                print("Target lost. Initiating search pattern...")
                self.robot_commander.stop()
                time.sleep(0.1)
                self.robot_commander.left()
                self.target_lost_search_active = True
                self.target_lost_search_start_time = current_time
                self.last_autonomous_action_time = current_time
            elif current_time - self.target_lost_search_start_time > 5:
                print("Search timeout. Stopping robot.")
                self.robot_commander.stop()
                self.target_lost_search_active = False
                self.autonomous_mode = False
            else:

                self.robot_commander.left()
                self.last_autonomous_action_time = current_time
            return

        if self.target_lost_search_active:
            print("Target reacquired. Stopping search pattern.")
            self.robot_commander.stop()
            self.target_lost_search_active = False

        self.last_autonomous_action_time = current_time

        print(f"Target Info - Angle: {angle:.1f} deg, Distance: {distance:.1f} px")

        if distance <= config.DISTANCE_THRESHOLD_PIXELS:
            self.robot_commander.stop()
            print("Target reached!")
            self.autonomous_mode = False
        elif abs(angle) > config.ANGLE_THRESHOLD_DEGREES:
            if angle > 0:
                self.robot_commander.left()
            else:
                self.robot_commander.right()
            time.sleep(config.TURN_COMMAND_DURATION_S)
        else:
            self.robot_commander.forward()
            time.sleep(config.FORWARD_COMMAND_DURATION_S)

        self.last_autonomous_action_time = time.time()

    def run_loop(self):
        if not self.camera_manager or not self.vision_processor:
            print("Components not initialized. Cannot run loop.")
            return

        rate_limiter = RateLimiter(15)

        while self.running:
            frame = self.camera_manager.get_frame()
            if frame is None:
                time.sleep(0.1)
                continue

            self.vision_processor.process_frame(frame.copy())