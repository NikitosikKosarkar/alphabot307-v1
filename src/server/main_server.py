import paho.mqtt.client as mqtt
import time
import signal
import threading
from .common_defs import (
    RobotCommand, string_to_robot_command,
    log_info, log_error, log_critical, log_warning
)
from .engine_controller import Engine

MQTT_BROKER_HOST = "192.168.1.104"  # Change to your MQTT broker
MQTT_BROKER_PORT = 1883
MQTT_COMMAND_TOPIC = "robot/command"
MQTT_CLIENT_ID = "robot_receiver_python"

robot_engine = Engine()
keep_running = True
mqtt_client = None
mqtt_connection_lock = threading.Lock()
is_mqtt_connected = False


def signal_handler(sig, frame):
    global keep_running
    log_warning(f"Получен сигнал {sig}, завершение работы робота...")
    keep_running = False
    if mqtt_client:
        mqtt_client.loop_stop()  # Stop network loop
        mqtt_client.disconnect()


def on_connect(client, userdata, flags, rc, properties=None):  # properties for MQTTv5
    global is_mqtt_connected
    with mqtt_connection_lock:
        if rc == 0:
            log_success("Успешное подключение к MQTT брокеру.")
            client.subscribe(MQTT_COMMAND_TOPIC)
            log_info(f"Подписка на топик: {MQTT_COMMAND_TOPIC}")
            is_mqtt_connected = True
        else:
            log_error(f"Не удалось подключиться к MQTT, код ошибки: {rc}")
            is_mqtt_connected = False


def on_disconnect(client, userdata, rc, properties=None):  # properties for MQTTv5
    global is_mqtt_connected
    with mqtt_connection_lock:
        is_mqtt_connected = False
    log_warning(f"Отключено от MQTT брокера. Код: {rc}")
    if rc != 0 and keep_running:  # Unexpected disconnect
        log_info("Попытка переподключения...")
        # Reconnect logic handled in main loop


def on_message(client, userdata, msg):
    command_str = msg.payload.decode()
    log_info(f"MQTT | Получена команда: '{command_str}' на топике '{msg.topic}'")

    cmd_enum = string_to_robot_command(command_str)

    if cmd_enum == RobotCommand.MOVE_FORWARD:
        robot_engine.forward()
    elif cmd_enum == RobotCommand.TURN_LEFT:
        robot_engine.turn_left()
    elif cmd_enum == RobotCommand.TURN_RIGHT:
        robot_engine.turn_right()
    elif cmd_enum == RobotCommand.STOP:
        robot_engine.stop()
    else:
        log_error(f"MQTT | Неизвестная команда: '{command_str}'")


def setup_mqtt_client():
    global mqtt_client
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=MQTT_CLIENT_ID)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message
    # mqtt_client.username_pw_set("user", "password") # If authentication is needed


def connect_mqtt():
    global is_mqtt_connected
    if mqtt_client is None:
        setup_mqtt_client()

    with mqtt_connection_lock:  # Protect is_mqtt_connected and connect call
        if is_mqtt_connected:
            return True
        try:
            log_info(f"Подключение к MQTT брокеру {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}...")
            mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
            mqtt_client.loop_start()  # Start network loop in background thread
            # Connection success/failure is handled by on_connect/on_disconnect
            # Give it a moment to establish connection
            # A more robust way would be to wait on an event set by on_connect
            time.sleep(1)  # Simplistic wait
            return is_mqtt_connected  # Check status after attempting
        except Exception as e:
            log_error(f"Ошибка при попытке подключения к MQTT: {e}")
            is_mqtt_connected = False
            return False


def main():
    global keep_running, mqtt_client
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    log_info("Запуск контроллера робота (Python)...")

    if not robot_engine.initialize_gpio():
        log_critical("Не удалось инициализировать GPIO. Выход.")
        return 1

    if not connect_mqtt():
        log_warning("Не удалось первоначальное подключение к MQTT. Будут попытки переподключения.")

    log_info(f"Робот слушает команды на MQTT {MQTT_BROKER_HOST} топик: {MQTT_COMMAND_TOPIC}")

    try:
        while keep_running:
            with mqtt_connection_lock:
                connected_status = is_mqtt_connected

            if not connected_status:
                log_error("MQTT соединение потеряно или не установлено. Попытка переподключения через 5 секунд...")
                time.sleep(5)
                if keep_running:  # Check if shutdown was signaled during sleep
                    connect_mqtt()  # Attempt to reconnect

            time.sleep(0.1)  # Main loop heartbeat

    except Exception as e:
        log_critical(f"Критическая ошибка в главном цикле робота: {e}")
    finally:
        log_info("Завершение работы контроллера робота...")
        if mqtt_client:
            mqtt_client.loop_stop()
            if is_mqtt_connected:  # Only try to disconnect if we think we are connected
                try:
                    mqtt_client.disconnect()
                    log_info("MQTT отключен.")
                except Exception as e:
                    log_error(f"Ошибка при отключении от MQTT: {e}")
        robot_engine.stop()
        robot_engine.cleanup_gpio()
        log_info("Контроллер робота остановлен.")


if __name__ == "__main__":
    main()

