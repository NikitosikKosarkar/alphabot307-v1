import paho.mqtt.client as mqtt
import config
import time


class MQTTClientHandler:
    def __init__(self, broker_ip, port, client_id):
        self.broker_ip = broker_ip
        self.port = port
        self.client_id = client_id
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect
        self.message_callback = None
        self.connected = False

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print(f"Successfully connected to MQTT broker at {self.broker_ip}")
            self.connected = True
            self.client.subscribe(config.TOPIC_LAPTOP_COMMANDS)
            print(f"Subscribed to {config.TOPIC_LAPTOP_COMMANDS}")
            self.client.subscribe(config.TOPIC_ROBOT_STATUS)
            print(f"Subscribed to {config.TOPIC_ROBOT_STATUS}")
        else:
            print(f"Failed to connect to MQTT broker: {mqtt.connack_string(reason_code)}")
            self.connected = False

    def _on_message(self, client, userdata, msg):
        if self.message_callback:
            try:
                payload = msg.payload.decode('utf-8')
                self.message_callback(msg.topic, payload)
            except Exception as e:
                print(f"Error decoding or processing message: {e}")
        else:
            print(f"Received message on {msg.topic} but no callback is set: {msg.payload.decode()}")

    def _on_disconnect(self, client, userdata, reason_code, properties):
        self.connected = False
        print(f"Disconnected from MQTT broker. Reason: {reason_code}")
        if reason_code != 0:
            print("Attempting to reconnect...")

    def set_message_callback(self, callback):
        self.message_callback = callback

    def connect(self):
        try:
            self.client.connect(self.broker_ip, self.port, 60)
        except Exception as e:
            print(f"MQTT connection error: {e}")
            return False
        return True

    def start_loop(self):
        self.client.loop_start()

    def stop_loop(self):
        self.client.loop_stop()

    def disconnect(self):
        self.client.disconnect()
        print("MQTT client disconnected.")

    def publish(self, topic, payload, qos=1, retain=False):
        if not self.connected:
            print("MQTT client not connected. Cannot publish.")
            return
        try:
            result = self.client.publish(topic, payload, qos=qos, retain=retain)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                print(f"Failed to publish message to {topic}: {mqtt.error_string(result.rc)}")


        except Exception as e:
            print(f"Error publishing message: {e}")