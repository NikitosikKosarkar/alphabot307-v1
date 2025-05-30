# AlphaBot Control System (Python Implementation)

This project contains Python implementations for controlling an AlphaBot robot:

1.  **AlphaBot Client**: An application (typically run on a machine with a camera, or on the Raspberry Pi if it handles both vision and motion) for image recognition and sending movement commands to the robot via MQTT.
2.  **AlphaBot Server**: An application running on the Raspberry Pi (AlphaBot) that receives MQTT commands and controls the robot's motors via GPIO using the `lgpio` library.

## Project Structure
