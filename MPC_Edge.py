#!/usr/bin/env python3

import time
import json
import requests
import paho.mqtt.client as mqtt

# ---------------------------------
# Home Assistant configuration
# ---------------------------------
HA_URL = "http://192.168.1.114:8123/api/template"
ha_access_token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJlM2U2ZGFjNjkwZGE0MDE0YjZkNWZmODJlYjUxODUwNCIsImlhdCI6MTczMTAxOTc1MiwiZXhwIjoyMDQ2Mzc5NzUyfQ.HLUT4rHcB0YlJxTFI_kOwDJqL5OETylGhT5nILvyyX0"

HA_HEADERS = {
    "Authorization": f"Bearer {ha_access_token}",
    "Content-Type": "application/json"
}

# ---------------------------------
# MQTT configuration
# ---------------------------------
MQTT_BROKER = "test.mosquitto.org"  # or your local broker IP
MQTT_PORT = 1883
MQTT_CLIENT_ID = "script1_publisher_subscriber"
TOPIC_SENSORS_RAW = "home/temperature/raw"
TOPIC_AVERAGE = "home/temperature/average"

# ---------------------------------
# Function to read from Home Assistant
# ---------------------------------
def get_indoor_temperature():
    """
    Retrieves three temperature values from Home Assistant and returns
    them as a tuple: (temp_1, temp_2, temp_3).
    Returns None if an error occurs.
    """
    payload = {
        "template": """
        {
            "indoor_temperature_1": "{{ states('sensor.h5100_566b_temperature') }}",
            "indoor_temperature_2": "{{ states('sensor.h5100_4d4d_temperature') }}",
            "indoor_temperature_3": "{{ states('sensor.h5100_3a46_temperature') }}"
        }
        """
    }

    try:
        response = requests.post(HA_URL, headers=HA_HEADERS, json=payload)
        response.raise_for_status()  # raise an error if not 200-299

        data = response.json()
        temp_1 = float(data['indoor_temperature_1'])
        temp_2 = float(data['indoor_temperature_2'])
        temp_3 = float(data['indoor_temperature_3'])

        return (temp_1, temp_2, temp_3)

    except (requests.RequestException, ValueError, KeyError) as e:
        print(f"[Script1] Error retrieving temperatures from Home Assistant: {e}")
        return None

# ---------------------------------
# MQTT callbacks
# ---------------------------------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[Script1] Connected to MQTT broker.")
        # Subscribe to the average temperature topic
        client.subscribe(TOPIC_AVERAGE)
        print(f"[Script1] Subscribed to: {TOPIC_AVERAGE}")
    else:
        print(f"[Script1] Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    """
    Handle incoming messages. Specifically:
    - For the TOPIC_AVERAGE, we print the average temperature.
    """
    if msg.topic == TOPIC_AVERAGE:
        average_temp_str = msg.payload.decode("utf-8")
        print(f"[Script1] Received average temperature: {average_temp_str}°C")

# ---------------------------------
# Main script
# ---------------------------------
if __name__ == "__main__":
    # Create MQTT client and assign callbacks
    client = mqtt.Client()  #MQTT_CLIENT_ID
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to the MQTT broker
    print("[Script1] Connecting to the MQTT broker...")
    client.connect(MQTT_BROKER, MQTT_PORT, 60)

    # Start the loop in a separate thread
    client.loop_start()

    # Give it a moment to connect
    time.sleep(2)

    # 1) Read temperatures from Home Assistant
    temps = get_indoor_temperature()
    if temps is not None:
        temp_1, temp_2, temp_3 = temps
        print(f"[Script1] Sensor readings: {temp_1}, {temp_2}, {temp_3}")

        # 2) Publish them as a JSON payload to TOPIC_SENSORS_RAW
        payload = {
            "temp_1": temp_1,
            "temp_2": temp_2,
            "temp_3": temp_3
        }
        client.publish(TOPIC_SENSORS_RAW, json.dumps(payload))
        print(f"[Script1] Published sensor values to {TOPIC_SENSORS_RAW}")

    else:
        print("[Script1] No sensor data to publish.")

    try:
        # Keep the script running to handle incoming messages (the average)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[Script1] Shutting down...")
    finally:
        client.loop_stop()
        client.disconnect()
