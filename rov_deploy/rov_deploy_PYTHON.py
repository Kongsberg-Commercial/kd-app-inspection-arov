import serial
import time
import paho.mqtt.client as mqtt

# --- Serial Configuration ---
SERIAL_PORT = 'COM5'  # Update to match your system, e.g., '/dev/ttyUSB0'
BAUD_RATE = 9600

# --- MQTT Configuration ---
MQTT_BROKER = '100.78.45.94'  # Or IP address of your MQTT broker
MQTT_PORT = 1883
MQTT_TOPIC = '/rov_deployment'
MQTT_PASSWORD = 'formula2boat'  # Optional, if your broker requires authentication
MQTT_USERNAME = 'formula2boat'  # Optional, if your broker requires authentication

# --- Connect to Serial ---
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for Arduino reset

def send_command_to_arduino(command):
    ser.write(f"{command}\n".encode())
    print(f"[Serial] Sent: {command}")

# --- MQTT Callbacks ---
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] Connected to broker")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"[MQTT] Connection failed with code {rc}")

def on_message(client, userdata, msg):
    try:    
        payload = msg.payload.decode().strip().lower()
        print(f"[MQTT] Received on {msg.topic}: {payload}")
        
        if payload == "deploy":
            send_command_to_arduino(-1)
        elif payload == "retrieve":
            send_command_to_arduino(1)
        elif payload == "stop":
            send_command_to_arduino(0)
        else:
            print(f"[MQTT] Unknown command: {payload}")
    except Exception as e:
        print(f"[Error] {e}")

# --- Start MQTT ---
client = mqtt.Client()
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)  # ← Add this line!
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_forever()
