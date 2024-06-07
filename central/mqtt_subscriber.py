import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import json
from collections import deque
from threading import Thread
# Configuration
BROKER_URL = "localhost"  # Update with your MQTT broker address
BROKER_PORT = 1883
TOPIC = "drone/telemetry"
COMMAND_TOPIC = "drone/commands"

graph_thread: Thread = None
# Initialize data storage
data = {
    'pitch': deque(maxlen=100),
    'roll': deque(maxlen=100),
    'yaw': deque(maxlen=100),
    'time': deque(maxlen=100)
}

# Initialize plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# Create the plane's body
body = np.array([[0.5, -0.5, 0],
                 [-0.5, -0.5, 0],
                 [-0.5, 0.5, 0],
                 [0.5, 0.5, 0],
                 [0.5, -0.5, 0]])

# Initialize the plot elements we want to animate
plane_body, = ax.plot([], [], [], 'b-')

def update(frame):
    if len(data['time']) == 0:
        return plane_body,

    pitch = np.deg2rad(data['pitch'][-1])
    roll = np.deg2rad(data['roll'][-1])
    yaw = np.deg2rad(data['yaw'][-1])

    # Create rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(pitch), -np.sin(pitch)],
                    [0, np.sin(pitch), np.cos(pitch)]])

    R_y = np.array([[np.cos(roll), 0, np.sin(roll)],
                    [0, 1, 0],
                    [-np.sin(roll), 0, np.cos(roll)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combine the rotation matrices
    R = np.dot(R_z, np.dot(R_y, R_x))

    # Rotate the body of the plane
    rotated_body = np.dot(body, R.T)

    # Update the plot data
    plane_body.set_data(rotated_body[:, 0], rotated_body[:, 1])
    plane_body.set_3d_properties(rotated_body[:, 2])

    return plane_body,

# MQTT callbacks
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    try:
        message = json.loads(payload)
        data['pitch'].append(message['pitch'])
        data['roll'].append(message['roll'])
        data['yaw'].append(message['yaw'])
        if len(data['time']) == 0:
            data['time'].append(0)
        else:
            data['time'].append(data['time'][-1] + 1)
    except json.JSONDecodeError:
        print("Failed to decode JSON message")

def send_command(client):
    while True:
        command = input("Enter command (start/full-stop: s): ").strip().lower()
        if command in ["start", "s", "restart"]:
            client.publish(COMMAND_TOPIC, command)
        elif command == "q":
            plt.close("all")
            if graph_thread is not None:                
                graph_thread.join()
            break
        else:
            print("Invalid command. Please enter 'start' or 'full stop'.")

# Set up MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_URL, BROKER_PORT, 60)
client.loop_start()

# Set up animation
ani = FuncAnimation(fig, update, blit=True, interval=100, cache_frame_data=False)
command_thread = Thread(target=send_command, args=(client,), daemon=True)
command_thread.start()
plt.show()


