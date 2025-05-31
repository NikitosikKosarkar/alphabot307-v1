import paho.mqtt.client as mqtt
import signal
import cv2
import numpy as np
import time
import sys


class RobotControlSystem:
    def __init__(self):
        self.broker_ip = "192.168.1.100"
        self.port = 1883
        self.robot_topic = "robot/gpio"

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect

        self.cap = self.initialize_camera()
        self.window_name = "Robot Vision Control"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)

        self.running = False
        self.vision_enabled = True
        self.show_camera = True
        self.front = (-1, -1)
        self.back = (-1, -1)
        self.target = (-1, -1)

    def initialize_camera(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("Failed to open camera")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        return cap

    def on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"Connected to MQTT broker at {self.broker_ip}")

    def send_gpio_command(self, command):
        try:
            self.client.publish(self.robot_topic, command)
            print(f"Command sent: {command}")
        except Exception as e:
            print(f"Command sending error: {e}")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to capture frame")
            return False

        frame = cv2.flip(frame, 1)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))  # H:0-10
        red_mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))  # H:170-180
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        blue_mask1 = cv2.inRange(hsv, np.array([100, 120, 50]), np.array([110, 255, 180]))  # H:100-110
        blue_mask2 = cv2.inRange(hsv, np.array([110, 120, 50]), np.array([120, 255, 180]))  # H:110-120
        blue_mask = cv2.bitwise_or(blue_mask1, blue_mask2)
        green_mask1 = cv2.inRange(hsv, np.array([35, 100, 60]), np.array([45, 255, 200]))  # H:35-45
        green_mask2 = cv2.inRange(hsv, np.array([45, 100, 60]), np.array([55, 255, 200]))  # H:45-55
        green_mask = cv2.bitwise_or(green_mask1, green_mask2)

        self.front = self.find_center(red_mask)
        self.back = self.find_center(blue_mask)
        self.target = self.find_center(green_mask)

        self.draw_detections(frame)

        if self.show_camera:
            cv2.imshow(self.window_name, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.shutdown()

        return True

    def draw_detections(self, frame):
        if self.front[0] >= 0 and self.back[0] >= 0:
            cv2.circle(frame, self.front, 10, (0, 0, 255), -1)
            cv2.circle(frame, self.back, 10, (255, 0, 0), -1)
            cv2.line(frame, self.back, self.front, (255, 255, 255), 2)
            cv2.putText(frame, "Front", (self.front[0] + 15, self.front[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(frame, "Back", (self.back[0] + 15, self.back[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        if self.target[0] >= 0:
            cv2.circle(frame, self.target, 15, (0, 255, 0), 2)
            cv2.putText(frame, "Target", (self.target[0] + 20, self.target[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if self.front[0] >= 0:
                cv2.line(frame, self.front, self.target, (0, 255, 0), 1)

    def find_center(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return (-1, -1)

        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        return (x + w // 2, y + h // 2)

    def get_angle_to_target(self):
        if self.front[0] < 0 or self.back[0] < 0 or self.target[0] < 0:
            return 360.0

        robot_dir = (self.front[0] - self.back[0], self.front[1] - self.back[1])
        to_target = (self.target[0] - self.front[0], self.target[1] - self.front[1])

        cross = robot_dir[0] * to_target[1] - robot_dir[1] * to_target[0]
        dot = robot_dir[0] * to_target[0] + robot_dir[1] * to_target[1]

        return np.arctan2(cross, dot) * 180 / np.pi

    def get_distance_to_target(self):
        if self.front[0] < 0 or self.target[0] < 0:
            return -1.0
        return np.sqrt((self.target[0] - self.front[0]) ** 2 + (self.target[1] - self.front[1]) ** 2)

    def vision_control_loop(self):
        angle_threshold = 5.0
        distance_threshold = 30.0

        while self.running and self.vision_enabled:
            if not self.process_frame():
                time.sleep(0.1)
                continue

            if self.target[0] < 0:
                print("Searching for target...")
                self.send_gpio_command("stop")
                time.sleep(0.1)
                continue

            angle = self.get_angle_to_target()
            distance = self.get_distance_to_target()

            print(f"Target info - Angle: {angle:.1f}°, Distance: {distance:.1f} px")

            if distance <= distance_threshold:
                self.send_gpio_command("stop")
                print("Target reached!")
                self.vision_enabled = False
            elif abs(angle) > angle_threshold:
                if angle > 0:
                    self.send_gpio_command("left")
                    time.sleep(0.1)
                else:
                    self.send_gpio_command("right")
                    time.sleep(0.1)
            else:
                self.send_gpio_command("forward")
                time.sleep(0.3)

    def start(self):
        try:
            self.client.connect(self.broker_ip, self.port, 60)
            self.running = True

            signal.signal(signal.SIGINT, self.shutdown)
            signal.signal(signal.SIGTERM, self.shutdown)

            while self.running:
                self.process_frame()
                if self.vision_enabled:
                    self.vision_control_loop()
                else:
                    time.sleep(0.05)

        except Exception as e:
            print(f"Startup error: {e}")
        finally:
            self.cleanup()

    def shutdown(self, signum=None, frame=None):
        print("\nShutting down system...")
        self.send_gpio_command("stop")
        self.running = False
        self.vision_enabled = False

    def cleanup(self):
        self.client.disconnect()
        self.cap.release()
        cv2.destroyAllWindows()
        print("System stopped")

if __name__ == "__main__":
    try:
        system = RobotControlSystem()
        system.start()
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)