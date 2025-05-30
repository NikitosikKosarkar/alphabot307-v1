import cv2
import numpy as np
import config
import math


class VisionProcessor:
    def __init__(self):
        self.front_pos = (-1, -1)
        self.back_pos = (-1, -1)
        self.target_pos = (-1, -1)

    def _find_center(self, mask, min_area=100):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return (-1, -1)

        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        if not valid_contours:
            return (-1, -1)

        largest_contour = max(valid_contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        return (x + w // 2, y + h // 2)

    def process_frame(self, frame):
        if frame is None:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_mask1 = cv2.inRange(hsv, np.array(config.RED_LOWER1), np.array(config.RED_UPPER1))
        red_mask2 = cv2.inRange(hsv, np.array(config.RED_LOWER2), np.array(config.RED_UPPER2))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        blue_mask = cv2.inRange(hsv, np.array(config.BLUE_LOWER), np.array(config.BLUE_UPPER))
        green_mask = cv2.inRange(hsv, np.array(config.GREEN_LOWER), np.array(config.GREEN_UPPER))

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        self.front_pos = self._find_center(red_mask)
        self.back_pos = self._find_center(blue_mask)
        self.target_pos = self._find_center(green_mask)

    def get_robot_orientation_vector(self):
        if self.front_pos[0] < 0 or self.back_pos[0] < 0:
            return None
        return (self.front_pos[0] - self.back_pos[0], self.front_pos[1] - self.back_pos[1])

    def get_vector_to_target(self):
        if self.front_pos[0] < 0 or self.target_pos[0] < 0:
            return None
        return (self.target_pos[0] - self.front_pos[0], self.target_pos[1] - self.front_pos[1])

    def get_angle_to_target(self):
        robot_dir = self.get_robot_orientation_vector()
        to_target_dir = self.get_vector_to_target()

        if not robot_dir or not to_target_dir:
            return None

        cross_product = robot_dir[0] * to_target_dir[1] - robot_dir[1] * to_target_dir[0]
        dot_product = robot_dir[0] * to_target_dir[0] + robot_dir[1] * to_target_dir[1]

        angle_rad = np.arctan2(cross_product, dot_product)
        return np.degrees(angle_rad)

    def get_distance_to_target(self):
        to_target_dir = self.get_vector_to_target()
        if not to_target_dir:
            return None
        return np.sqrt(to_target_dir[0] ** 2 + to_target_dir[1] ** 2)

    def draw_detections(self, frame, vision_mode_status="MANUAL"):
        if frame is None:
            return frame

        if self.front_pos[0] >= 0:
            cv2.circle(frame, self.front_pos, 10, (0, 0, 255), -1)
            cv2.putText(frame, "Front", (self.front_pos[0] + 15, self.front_pos[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        if self.back_pos[0] >= 0:
            cv2.circle(frame, self.back_pos, 10, (255, 0, 0), -1)
            cv2.putText(frame, "Back", (self.back_pos[0] + 15, self.back_pos[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        if self.front_pos[0] >= 0 and self.back_pos[0] >= 0:
            cv2.line(frame, self.back_pos, self.front_pos, (255, 255, 255), 2)

        if self.target_pos[0] >= 0:
            cv2.circle(frame, self.target_pos, 15, (0, 255, 0), 2)
            cv2.putText(frame, "Target", (self.target_pos[0] + 20, self.target_pos[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if self.front_pos[0] >= 0:
                cv2.line(frame, self.front_pos, self.target_pos, (0, 255, 0), 1)

        cv2.putText(frame, f"Mode: {vision_mode_status}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        angle = self.get_angle_to_target()
        distance = self.get_distance_to_target()

        if angle is not None:
            cv2.putText(frame, f"Angle: {angle:.1f} deg", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
        if distance is not None:
            cv2.putText(frame, f"Dist: {distance:.1f} px", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

        return frame