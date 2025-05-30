import cv2
import config
import time


class CameraManager:
    def __init__(self, camera_index=config.CAMERA_INDEX, width=config.FRAME_WIDTH, height=config.FRAME_HEIGHT,
                 fps=config.FPS):
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera at index {camera_index}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Camera initialized. Requested: {width}x{height} @ {fps}fps. Actual: {actual_width}x{actual_height}")

        self.window_name = config.WINDOW_NAME
        self.show_display = True
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, config.WINDOW_WIDTH, config.WINDOW_HEIGHT)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to capture frame from camera.")
            return None
        return cv2.flip(frame, 1)

    def display_frame(self, frame):
        if self.show_display and frame is not None:
            cv2.imshow(self.window_name, frame)
        elif not self.show_display:

            black_screen = cv2.UMat(config.FRAME_HEIGHT, config.FRAME_WIDTH, cv2.CV_8UC3)
            black_screen.setTo(cv2.Scalar(0, 0, 0))
            cv2.putText(black_screen.get(), "Camera Display Disabled (Press 'c')", (50, config.FRAME_HEIGHT // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.imshow(self.window_name, black_screen.get())

    def toggle_display(self):
        self.show_display = not self.show_display
        if not self.show_display:

            print(f"Camera display {'enabled' if self.show_display else 'disabled'}")
        else:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, config.WINDOW_WIDTH, config.WINDOW_HEIGHT)
            print(f"Camera display {'enabled' if self.show_display else 'disabled'}")

    def process_key_input(self):
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            return 'quit'
        elif key == ord('v'):
            return 'toggle_vision'
        elif key == ord('c'):
            return 'toggle_camera'
        elif key == ord('s'):
            return 'stop'
        elif key == ord('w'):
            return 'forward'
        elif key == ord('a'):
            return 'left'
        elif key == ord('d'):
            return 'right'
        elif key == ord('x'):
            return 'backward'
        return None

    def release(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        print("Camera resources released.")
