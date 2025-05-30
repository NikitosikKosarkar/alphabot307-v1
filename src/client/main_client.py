import os
import time
import logging
import sys
from .robot_commander import RobotCommander, RobotStatus
from .image_recognition import get_angle, cleanup_display

# Configure basic logging for the client
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s][%(name)s] %(asctime)s %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("AlphaBotClient.Main")


def get_env_var(name: str, default_value: str) -> str:
    return os.environ.get(name, default_value)


def main():
    broker_address = get_env_var("BROKER_ADDRESS", "192.168.1.104")  # Match server or your broker
    try:
        broker_port = int(get_env_var("BROKER_PORT", "1883"))
    except ValueError:
        logger.warning("Invalid BROKER_PORT, using default 1883.")
        broker_port = 1883

    # This topic is NOT used by RobotCommander as it has its own fixed topic
    # to match the server. Kept here for consistency with original C++ main.
    # _ = get_env_var("COMMANDS_TOPIC", "/devices/wb-adc/controls")

    # The RobotCommander now uses "robot/command" topic by default.
    robot = RobotCommander(broker_address, broker_port)

    # Time to allow MQTT connection to establish if it's async
    time.sleep(2)

    camera_idx = 2  # As per C++ code default
    display_cv_window = True  # Set to False to run headless

    logger.info("Starting AlphaBot Client (Python)...")
    try:
        while True:
            # The C++ client had a loop `while (robot.getStatus() == StatusEnum::RUNNING)`
            # This effectively made processing synchronous for each command.
            # RobotCommander's _send_command has a small sleep, and sets status to IDLE quickly.
            # The main loop now dictates the pace.
            if robot.get_status() == RobotStatus.RUNNING:
                time.sleep(0.05)  # Wait a bit if robot is "busy" (short status)
                continue

            # Process image and get angle
            # Pass display_window=False to run headless
            angle = get_angle(camera_index=camera_idx, display_window=display_cv_window)
            logger.info(f"Calculated angle: {angle:.1f} degrees")

            # Logic from C++ client's main
            if angle == -360.0:  # Special condition to stop and exit
                logger.info("Lost target (green marker). Stopping and exiting.")
                robot.stop()
                break

                # Thresholds for turning
            if angle > 5.0:
                robot.left(int(angle))
            elif angle < -5.0:
                robot.right(int(angle))
            else:  # If angle is within -5 to 5 degrees, move forward
                robot.forward()

            # Add a small delay to control the loop speed if get_angle is very fast
            # time.sleep(0.1) # Optional: control processing rate

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received. Stopping robot and exiting.")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        logger.info("Cleaning up...")
        robot.stop()  # Send a final stop command
        robot.disconnect()
        if display_cv_window:
            cleanup_display()  # Close OpenCV windows
        logger.info("AlphaBot Client stopped.")


if __name__ == "__main__":
    main()
