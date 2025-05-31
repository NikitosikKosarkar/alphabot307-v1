import cv2
import numpy as np
import math
import paho.mqtt.client as mqtt
import threading
import signal
import time
import sys


class CameraProcessor:
    DEFAULT_HSV_RANGES = {
        "green": [40, 40, 40, 95, 255, 255],
        "pink": [140, 60, 80, 175, 255, 255],
        "blue": [95, 80, 80, 128, 255, 255],
    }

    OUTPUT_WINDOW_NAME = "Camera Debug Output"

    KERNEL_MORPH_OPEN = np.ones((5, 5), np.uint8)
    KERNEL_MORPH_CLOSE_5x5 = np.ones((5, 5), np.uint8)
    KERNEL_MORPH_CLOSE_7x7 = np.ones((7, 7), np.uint8)

    def __init__(self, process_frame_width=640, debug=False, initial_hsv_ranges=None):
        self.process_frame_width = process_frame_width
        self._debug_mode = False

        if initial_hsv_ranges is None:
            self.hsv_ranges = {k: list(v) for k, v in self.DEFAULT_HSV_RANGES.items()}
        else:
            self.hsv_ranges = {k: list(v) for k, v in initial_hsv_ranges.items()}

        self.debug_mode = debug

    @property
    def debug_mode(self):
        return self._debug_mode

    @debug_mode.setter
    def debug_mode(self, value):
        if self._debug_mode == value:
            return
        self._debug_mode = value
        if self._debug_mode:
            self._setup_debug_windows()
        else:
            self.release_windows()

    def _setup_debug_windows(self):
        cv2.startWindowThread()
        cv2.namedWindow(self.OUTPUT_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.OUTPUT_WINDOW_NAME, 960, 720)

    def _find_largest_contour_and_centroid(self, mask, min_area=30):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None

        largest_contour = None
        max_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area > min_area and area > max_area:
                max_area = area
                largest_contour = c

        if largest_contour is None:
            return None, None

        M = cv2.moments(largest_contour)
        if M["m00"] == 0:
            return None, None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy), largest_contour

    def get_processing_results(self, original_img, base_min_area_scale_factor=1.0):
        scale_ratio = 1.0
        if self.process_frame_width and original_img.shape[1] > self.process_frame_width:
            scale_ratio = self.process_frame_width / original_img.shape[1]
            height = int(original_img.shape[0] * scale_ratio)
            processed_img = cv2.resize(original_img, (self.process_frame_width, height), interpolation=cv2.INTER_AREA)
        else:
            processed_img = original_img.copy()

        current_min_area_scale = (scale_ratio ** 2) * base_min_area_scale_factor

        output_img_for_debug = processed_img.copy() if self.debug_mode else None
        hsv_img = cv2.cvtColor(processed_img, cv2.COLOR_BGR2HSV)

        current_hsv_cv = {}
        for color, values in self.hsv_ranges.items():
            current_hsv_cv[color] = (np.array(values[0:3]), np.array(values[3:6]))

        min_area_pink_blue = int(50 * current_min_area_scale)
        min_area_green = int(100 * current_min_area_scale)

        lower_pink, upper_pink = current_hsv_cv["pink"]
        mask_pink = cv2.inRange(hsv_img, lower_pink, upper_pink)
        mask_pink = cv2.morphologyEx(mask_pink, cv2.MORPH_OPEN, self.KERNEL_MORPH_OPEN)
        mask_pink = cv2.morphologyEx(mask_pink, cv2.MORPH_CLOSE, self.KERNEL_MORPH_CLOSE_5x5)
        centroid_pink, _ = self._find_largest_contour_and_centroid(mask_pink, min_area=min_area_pink_blue)

        lower_blue, upper_blue = current_hsv_cv["blue"]
        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, self.KERNEL_MORPH_OPEN)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, self.KERNEL_MORPH_CLOSE_5x5)
        centroid_blue, _ = self._find_largest_contour_and_centroid(mask_blue, min_area=min_area_pink_blue)

        robot_center = None
        robot_heading_rad = None
        distance_to_target = None
        angle_to_target_deg = None

        if self.debug_mode and output_img_for_debug is not None:
            if centroid_pink:
                cv2.circle(output_img_for_debug, centroid_pink, 5, (203, 192, 255), -1)
            if centroid_blue:
                cv2.circle(output_img_for_debug, centroid_blue, 5, (255, 192, 203), -1)

        if centroid_pink and centroid_blue:
            if self.debug_mode and output_img_for_debug is not None:
                cv2.line(output_img_for_debug, centroid_pink, centroid_blue, (255, 0, 255), 2)

            robot_center_x = (centroid_pink[0] + centroid_blue[0]) // 2
            robot_center_y = (centroid_pink[1] + centroid_blue[1]) // 2
            robot_center = (robot_center_x, robot_center_y)
            if self.debug_mode and output_img_for_debug is not None:
                cv2.circle(output_img_for_debug, robot_center, 7, (255, 255, 0), -1)

            robot_dx = centroid_pink[0] - centroid_blue[0]
            robot_dy = centroid_pink[1] - centroid_blue[1]
            robot_heading_rad = math.atan2(-robot_dy, robot_dx)

        lower_green, upper_green = current_hsv_cv["green"]
        mask_green = cv2.inRange(hsv_img, lower_green, upper_green)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, self.KERNEL_MORPH_OPEN)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, self.KERNEL_MORPH_CLOSE_7x7)
        centroid_green, _ = self._find_largest_contour_and_centroid(mask_green, min_area=min_area_green)

        if self.debug_mode and output_img_for_debug is not None and centroid_green:
            cv2.circle(output_img_for_debug, centroid_green, 7, (0, 255, 0), -1)

        if robot_center and centroid_green:
            if self.debug_mode and output_img_for_debug is not None:
                cv2.line(output_img_for_debug, robot_center, centroid_green, (0, 255, 255), 2)

            distance_to_target = math.hypot(centroid_green[0] - robot_center[0], centroid_green[1] - robot_center[1])

            target_dx = centroid_green[0] - robot_center[0]
            target_dy = centroid_green[1] - robot_center[1]
            world_angle_to_target_rad = math.atan2(-target_dy, target_dx)

            if robot_heading_rad is not None:
                steer_angle_rad = world_angle_to_target_rad - robot_heading_rad
                steer_angle_rad = (steer_angle_rad + math.pi) % (2 * math.pi) - math.pi
                angle_to_target_deg = math.degrees(steer_angle_rad)
            else:
                angle_to_target_deg = math.degrees(world_angle_to_target_rad)

            if self.debug_mode and output_img_for_debug is not None:
                mid_line_x = (robot_center[0] + centroid_green[0]) // 2
                mid_line_y = (robot_center[1] + centroid_green[1]) // 2

                dist_text = f"Dist: {distance_to_target:.0f}px"
                angle_text = f"Angle: {angle_to_target_deg:.0f}deg"

                font_scale = 0.5
                thickness = 1
                cv2.putText(output_img_for_debug, dist_text, (mid_line_x + 5, mid_line_y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 1, cv2.LINE_AA)
                cv2.putText(output_img_for_debug, dist_text, (mid_line_x + 5, mid_line_y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

                cv2.putText(output_img_for_debug, angle_text, (mid_line_x + 5, mid_line_y + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 1, cv2.LINE_AA)
                cv2.putText(output_img_for_debug, angle_text, (mid_line_x + 5, mid_line_y + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

        if self.debug_mode and output_img_for_debug is not None:
            status_text_on_debug = f"Robot Vision Active: {self.parent_vision_enabled if hasattr(self, 'parent_vision_enabled') else 'N/A'}"
            cv2.putText(output_img_for_debug, status_text_on_debug, (10, processed_img.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(output_img_for_debug, status_text_on_debug, (10, processed_img.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(self.OUTPUT_WINDOW_NAME, output_img_for_debug)

        results = {
            "robot_center_uv": robot_center,
            "robot_heading_rad": robot_heading_rad,
            "target_center_uv": centroid_green,
            "distance_px": distance_to_target,
            "angle_to_target_deg": angle_to_target_deg,
            "scale_ratio": scale_ratio,
            "debug_image": output_img_for_debug
        }
        return results

    def get_current_hsv_ranges(self):
        return self.hsv_ranges

    def release_windows(self):
        if cv2.getWindowProperty(self.OUTPUT_WINDOW_NAME, cv2.WND_PROP_VISIBLE) >= 1:
            cv2.destroyWindow(self.OUTPUT_WINDOW_NAME)


class RobotControlSystem:
    def __init__(self):
        self.broker_ip = "192.168.1.102"
        self.port = 1883
        self.robot_topic = "robot/gpio"

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect

        self.cap = self.initialize_camera()

        self.show_camera_debug_feed = True
        self.camera_processor = CameraProcessor(debug=self.show_camera_debug_feed, process_frame_width=640)

        self.running = False
        self.vision_enabled = False

        self.angle_threshold_deg = 10.0
        self.distance_threshold_px = 50.0
        self.turn_duration_s = 0.05
        self.forward_duration_s = 0.1

    def initialize_camera(self):
        for i in range(3, -1, -1):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"Camera opened successfully at index {i}")
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                return cap
        raise RuntimeError("Failed to open any camera. Ensure it is connected and not in use.")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print(f"Connected to MQTT broker at {self.broker_ip}")
        else:
            print(f"Failed to connect to MQTT broker: {reason_code}")

    def send_gpio_command(self, command):
        try:
            self.client.publish(self.robot_topic, command.encode('utf-8'))
        except Exception as e:
            print(f"Error sending MQTT command: {e}")

    def start(self):
        try:
            self.client.connect(self.broker_ip, self.port, 60)
            self.running = True

            signal.signal(signal.SIGINT, self.handle_shutdown_signal)
            signal.signal(signal.SIGTERM, self.handle_shutdown_signal)

            mqtt_thread = threading.Thread(target=self.client.loop_forever)
            mqtt_thread.daemon = True
            mqtt_thread.start()

            print("Robot Control System Started.")
            print("------------------------------------")
            print("Keyboard Controls:")
            print("  'v': Toggle Autonomous Vision Mode")
            print("  'c': Toggle Camera Debug Feed")
            print("  'q': Quit")
            print("------------------------------------")

            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    print("Error: Failed to capture frame from camera. Retrying...")
                    self.cap.release()
                    time.sleep(1)
                    self.cap = self.initialize_camera()
                    if not self.cap.isOpened():
                        print("FATAL: Could not re-initialize camera. Shutting down.")
                        self.running = False
                        break
                    continue

                frame = cv2.flip(frame, 1)

                if hasattr(self.camera_processor, 'parent_vision_enabled'):
                    self.camera_processor.parent_vision_enabled = self.vision_enabled

                results = self.camera_processor.get_processing_results(frame)

                if self.vision_enabled:
                    robot_center = results.get("robot_center_uv")
                    target_center = results.get("target_center_uv")
                    angle_deg = results.get("angle_to_target_deg")
                    distance_px = results.get("distance_px")

                    if not robot_center or not target_center:
                        self.send_gpio_command("stop")
                        time.sleep(0.1)
                    elif distance_px is None or angle_deg is None:
                        self.send_gpio_command("stop")
                        time.sleep(0.1)
                    else:
                        if distance_px <= self.distance_threshold_px:
                            self.send_gpio_command("stop")
                            print(f"Vision: Target reached (Dist: {distance_px:.1f}px). Disabling autonomous mode.")
                            self.vision_enabled = False
                        elif abs(angle_deg) > self.angle_threshold_deg:
                            if angle_deg > 0:
                                self.send_gpio_command("left")
                                time.sleep(self.turn_duration_s)
                            else:
                                self.send_gpio_command("right")
                                time.sleep(self.turn_duration_s)
                        else:
                            self.send_gpio_command("forward")
                            time.sleep(self.forward_duration_s)
                else:
                    time.sleep(0.03)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("'q' pressed, initiating shutdown...")
                    self.shutdown()
                elif key == ord('v'):
                    self.vision_enabled = not self.vision_enabled
                    print(f"Autonomous Vision Mode: {'ENABLED' if self.vision_enabled else 'DISABLED'}")
                    if not self.vision_enabled:
                        self.send_gpio_command("stop")
                elif key == ord('c'):
                    self.camera_processor.debug_mode = not self.camera_processor.debug_mode
                    print(f"Camera Debug Feed: {'ENABLED' if self.camera_processor.debug_mode else 'DISABLED'}")

        except Exception as e:
            print(f"An error occurred in the main loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("Cleaning up resources...")
            self.cleanup()

    def handle_shutdown_signal(self, signum, frame):
        print(f"Received signal {signum}. Initiating shutdown...")
        self.shutdown()

    def shutdown(self):
        if not self.running:
            return
        print("Shutting down system...")
        self.running = False
        self.vision_enabled = False

        time.sleep(0.2)

        if self.client.is_connected():
            self.send_gpio_command("stop")
            print("Final stop command sent.")

    def cleanup(self):
        print("Starting cleanup...")
        if self.client.is_connected():
            self.client.loop_stop(force=False)
            self.client.disconnect()
            print("Disconnected from MQTT broker.")

        if self.cap and self.cap.isOpened():
            self.cap.release()
            print("Camera released.")

        self.camera_processor.release_windows()
        cv2.destroyAllWindows()
        print("OpenCV windows destroyed.")
        print("System stopped.")


if __name__ == "__main__":
    try:
        system = RobotControlSystem()
        system.camera_processor.parent_vision_enabled = system.vision_enabled
        system.start()
    except RuntimeError as e:
        print(f"Runtime Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback

        traceback.print_exc()
    finally:
        print("Application exiting.")
        sys.exit(0)