import paho.mqtt.client as mqtt
import time
import enum
import logging
import threading

logger = logging.getLogger("AlphaBotClient.RobotCommander")


class CommandName(enum.Enum):
    FORWARD = "forward"
    LEFT = "left"
    RIGHT = "right"
    STOP = "stop"


class RobotStatus(enum.Enum):
    IDLE = 0
    RUNNING = 1


class RobotCommander:
    def __init__(self, broker_address: str, broker_port: int, command_topic: str = "robot/command"):
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.command_topic = command_topic  # Server expects this topic

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="robot_commander_python")
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self._connected = False
        self._connection_lock = threading.Lock()

        self.status = RobotStatus.IDLE
        # Action duration is handled by server now, client just sends commands
        # self.default_action_duration_ms = 200 # From C++ client, but server has its own default

        try:
            logger.info(f"Attempting to connect to MQTT broker at {broker_address}:{broker_port}")
            self.client.connect(broker_address, broker_port, 60)
            self.client.loop_start()  # Start network loop
        except Exception as e:
            logger.error(f"Failed to connect to MQTT broker on init: {e}")

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        with self._connection_lock:
            if rc == 0:
                logger.info("Successfully connected to MQTT broker.")
                self._connected = True
            else:
                logger.error(f"Failed to connect to MQTT broker, return code {rc}")
                self._connected = False

    def _on_disconnect(self, client, userdata, rc, properties=None):
        with self._connection_lock:
            self._connected = False
        logger.warning(f"Disconnected from MQTT broker. RC: {rc}")
        # Implement reconnection logic if desired, or handle in main loop

    def _send_command(self, command_name: CommandName):
        if not self._connected:
            logger.warning("MQTT client not connected. Command not sent.")
            # Optionally try to reconnect here
            # try:
            #     self.client.reconnect()
            # except Exception as e:
            #     logger.error(f"Failed to reconnect: {e}")
            return

        message = command_name.value
        topic = self.command_topic  # Use the topic server is listening to

        logger.info(f"Publishing command: '{message}' to topic '{topic}'")
        self.status = RobotStatus.RUNNING  # Client sets its status
        try:
            self.client.publish(topic, message)
        except Exception as e:
            logger.error(f"Error publishing MQTT message: {e}")
            self.status = RobotStatus.IDLE  # Revert status on failure
            return

        # The C++ client had a complicated command queue and thread management.
        # For this Python version, we simplify. The main loop in `main_client.py`
        # will control the rate of commands.
        # This small delay can prevent overwhelming the system if get_angle is very fast.
        # The actual duration of the robot's movement is controlled by the SERVER.
        time.sleep(0.1)  # Small delay after sending command
        self.status = RobotStatus.IDLE

    def left(self, angle_degrees: int = 0):  # angle_degrees currently unused by command
        logger.info(f"Command: LEFT (angle: {angle_degrees})")
        self._send_command(CommandName.LEFT)

    def right(self, angle_degrees: int = 0):  # angle_degrees currently unused by command
        logger.info(f"Command: RIGHT (angle: {angle_degrees})")
        self._send_command(CommandName.RIGHT)

    def forward(self, duration_ms: int = 0):  # duration_ms currently unused by command
        logger.info(f"Command: FORWARD (duration: {duration_ms}ms)")
        self._send_command(CommandName.FORWARD)

    def stop(self, duration_ms: int = 0):  # duration_ms currently unused by command
        logger.info(f"Command: STOP (duration: {duration_ms}ms)")
        self._send_command(CommandName.STOP)

    def get_status(self) -> RobotStatus:
        return self.status

    def disconnect(self):
        logger.info("Disconnecting MQTT client.")
        self.client.loop_stop()
        self.client.disconnect()

    def __del__(self):
        if self.client and self._connected:
            self.disconnect()

