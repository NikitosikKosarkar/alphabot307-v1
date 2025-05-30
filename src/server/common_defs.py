import enum
import logging
import sys

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(asctime)s %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)

class WheelState(enum.Enum):
    FORWARD = 1
    REVERSE = 2
    STOP = 0

class Wheel(enum.Enum):
    LEFT = 0
    RIGHT = 1

class RobotCommand(enum.Enum):
    MOVE_FORWARD = "forward"
    TURN_LEFT = "left"
    TURN_RIGHT = "right"
    STOP = "stop"
    UNKNOWN = "unknown"

def string_to_robot_command(cmd_str: str) -> RobotCommand:
    for command in RobotCommand:
        if command.value == cmd_str:
            return command
    return RobotCommand.UNKNOWN

DEFAULT_ACTION_DURATION_S = 0.25

logger = logging.getLogger("AlphaBotServer")

def log_info(msg):
    logger.info(msg)

def log_error(msg):
    logger.error(msg)

def log_success(msg): # Custom level or just use info
    logger.info(f"SUCCESS: {msg}")

def log_warning(msg):
    logger.warning(msg)

def log_critical(msg):
    logger.critical(msg)