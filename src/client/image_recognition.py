import cv2
import numpy as np
import math
import logging

logger = logging.getLogger("AlphaBotClient.ImageRecognition")

# Ensure M_PI is defined if not available (e.g. on Windows math module)
try:
    M_PI = math.pi
except AttributeError:
    M_PI = 3.14159265358979323846


def _get_center_coordinates_internal(mask):
    kernel_erode = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 4))
    eroded_mask = cv2.erode(mask, kernel_erode)

    kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6))
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate)

    contours, _ = cv2.findContours(dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return 0, 0

    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    if not contours[0].size:  # Check if the largest contour is not empty
        return 0, 0

    M = cv2.moments(contours[0])
    if M["m00"] == 0.0:
        return 0, 0

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy


def _angle_between_segments_internal(a1, a2, b1, b2):
    # Convert to numpy arrays for easier math
    pt_a1 = np.array(a1, dtype=float)
    pt_a2 = np.array(a2, dtype=float)
    pt_b1 = np.array(b1, dtype=float)
    pt_b2 = np.array(b2, dtype=float)

    vec_a = pt_a2 - pt_a1
    vec_b = pt_b2 - pt_b1

    ax, ay = vec_a[0], vec_a[1]
    bx, by = vec_b[0], vec_b[1]

    dot = ax * bx + ay * by
    len_a = np.linalg.norm(vec_a)
    len_b = np.linalg.norm(vec_b)

    if len_a == 0.0 or len_b == 0.0:
        return 0.0

    cos_val = dot / (len_a * len_b)
    cos_val = np.clip(cos_val, -1.0, 1.0)  # Ensure value is in range for acos

    angle_rad = np.arccos(cos_val)

    cross = ax * by - ay * bx
    if cross > 0:  # Determine sign based on cross product (orientation)
        angle_rad = -angle_rad

    return angle_rad * 180.0 / M_PI


def get_angle(camera_index=2, display_window=True):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        logger.error(f"Cannot open camera index {camera_index}")
        return 0.0

    window_name = "AlphaBot View"
    if display_window:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        # cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    ret, frame = cap.read()
    angle = 0.0

    if not ret or frame is None:
        logger.error("Failed to capture frame from camera")
        cap.release()
        if display_window:
            cv2.destroyAllWindows()
        return 0.0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    try:
        # HSV color ranges
        lower_blue = np.array([90, 150, 0])
        upper_blue = np.array([150, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_blue_center_coords = _get_center_coordinates_internal(mask_blue)

        lower_pink = np.array([130, 150, 0])  # Values might need adjustment
        upper_pink = np.array([180, 255, 255])
        mask_pink = cv2.inRange(hsv, lower_pink, upper_pink)
        mask_pink_center_coords = _get_center_coordinates_internal(mask_pink)

        lower_green = np.array([30, 70, 0])
        upper_green = np.array([100, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_green_center_coords = _get_center_coordinates_internal(mask_green)

        result_frame = frame.copy()  # Draw on a copy

        # For visualization (optional, but helpful for debugging)
        # result_blue = cv2.bitwise_and(result_frame, result_frame, mask=mask_blue)
        # result_pink = cv2.bitwise_and(result_frame, result_frame, mask=mask_pink)
        # result_green = cv2.bitwise_and(result_frame, result_frame, mask=mask_green)
        # combined_results = cv2.bitwise_or(result_blue, result_pink)
        # combined_results = cv2.bitwise_or(combined_results, result_green)

        cv2.circle(result_frame, mask_blue_center_coords, 20, (0, 0, 255), -1)
        cv2.circle(result_frame, mask_pink_center_coords, 20, (0, 0, 255),
                   -1)  # Pink is often BGR(255,0,255) or similar
        cv2.circle(result_frame, mask_green_center_coords, 20, (0, 0, 255), -1)  # Green is (0,255,0)

        cv2.line(result_frame, mask_blue_center_coords, mask_pink_center_coords, (255, 255, 255), 10)

        line_center_coords = (
            (mask_blue_center_coords[0] + mask_pink_center_coords[0]) // 2,
            (mask_blue_center_coords[1] + mask_pink_center_coords[1]) // 2
        )

        cv2.circle(result_frame, line_center_coords, 20, (0, 0, 255), -1)  # Red
        cv2.line(result_frame, line_center_coords, mask_green_center_coords, (255, 255, 255), 10)  # White

        # Calculate angle
        # Segment 1: mask_blue_center_coords -> line_center_coords
        # Segment 2: mask_green_center_coords -> line_center_coords
        # (Order might matter for angle sign depending on _angle_between_segments_internal implementation)
        # The C++ code used:
        # (mask_blue_center_coords, line_center_coords)
        # (mask_green_center_coords, line_center_coords)
        # This means the common point is line_center_coords for the angle vertex
        angle = float(_angle_between_segments_internal(
            mask_blue_center_coords, line_center_coords,
            mask_green_center_coords, line_center_coords
        ))

        cv2.putText(result_frame, str(int(angle)),
                    (line_center_coords[0], line_center_coords[1] + 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)  # Red text, increased size

        if display_window:
            cv2.imshow(window_name, result_frame)
            cv2.waitKey(1)  # Essential for imshow to refresh

        # Special condition from C++ client
        if mask_green_center_coords[0] == 0 and mask_green_center_coords[1] == 0:
            angle = -360.0

    except cv2.error as e:
        logger.error(f"OpenCV error in image processing: {e}")
        angle = 0.0
    except Exception as e:
        logger.error(f"Generic error in image processing: {e}")
        angle = 0.0
    finally:
        cap.release()
        # Don't destroy windows here if used in a loop, only at the end of application
        # if display_window:
        #     cv2.destroyAllWindows() # Or just cv2.destroyWindow(window_name)

    return angle


def cleanup_display():
    cv2.destroyAllWindows()

