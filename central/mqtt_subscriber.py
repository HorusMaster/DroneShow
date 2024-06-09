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
    'altitude': deque(maxlen=100),
    'motor1': deque(maxlen=100),
    'motor2': deque(maxlen=100),
    'motor3': deque(maxlen=100),
    'motor4': deque(maxlen=100),
    'pidpitch': deque(maxlen=100),
    'pidroll': deque(maxlen=100),
    'pidyaw': deque(maxlen=100),
    'pidalt': deque(maxlen=100),
    'time': deque(maxlen=100)
}

# Initialize plot
fig = plt.figure(figsize=(14, 14))

# 3D plot for the drone orientation
ax_3d = fig.add_subplot(221, projection='3d')
ax_3d.set_xlim([-1, 1])
ax_3d.set_ylim([-1, 1])
ax_3d.set_zlim([-1, 1])

# Create the plane's body
body = np.array([[0.5, -0.5, 0],
                 [-0.5, -0.5, 0],
                 [-0.5, 0.5, 0],
                 [0.5, 0.5, 0],
                 [0.5, -0.5, 0]])

# Initialize the plot elements we want to animate
plane_body, = ax_3d.plot([], [], [], 'b-')

# Subplots for pitch, roll, and yaw
ax_pitch = fig.add_subplot(222)
ax_roll = fig.add_subplot(223)
ax_yaw = fig.add_subplot(224)

# Initialize text object
info_text = ax_3d.text2D(0.05, 0.95, '', transform=ax_3d.transAxes, verticalalignment='top', fontsize=12)

def update(frame):
    if len(data['time']) == 0:
        return plane_body, info_text

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

    # Update the 3D plot data
    plane_body.set_data(rotated_body[:, 0], rotated_body[:, 1])
    plane_body.set_3d_properties(rotated_body[:, 2])

    # Update the text annotations
    text_str = f"""Pitch: {data['pitch'][-1]:.2f}°
Roll: {data['roll'][-1]:.2f}°
Yaw: {data['yaw'][-1]:.2f}°
Altitude: {data['altitude'][-1]:.2f}m

Motor1: {data['motor1'][-1]:.2f}
Motor2: {data['motor2'][-1]:.2f}
Motor3: {data['motor3'][-1]:.2f}
Motor4: {data['motor4'][-1]:.2f}

PID Pitch: {data['pidpitch'][-1]:.2f}
PID Roll: {data['pidroll'][-1]:.2f}
PID Yaw: {data['pidyaw'][-1]:.2f}
PID Altitude: {data['pidalt'][-1]:.2f}
"""
    info_text.set_text(text_str)

    # Update the 2D plots
    ax_pitch.clear()
    ax_roll.clear()
    ax_yaw.clear()

    ax_pitch.plot(data['time'], data['pitch'], label='Pitch')
    ax_roll.plot(data['time'], data['roll'], label='Roll')
    ax_yaw.plot(data['time'], data['yaw'], label='Yaw')

    # Set static limits for the plots
    ax_pitch.set_ylim([-20, 20])
    ax_roll.set_ylim([-20, 20])
    ax_yaw.set_ylim([-20, 20])

    # Dynamically adjust the x-axis scale to fit the current data
    if len(data['time']) > 1:
        x_min = max(0, data['time'][-1] - 100)
        x_max = data['time'][-1]
        if x_min != x_max:
            ax_pitch.set_xlim([x_min, x_max])
            ax_roll.set_xlim([x_min, x_max])
            ax_yaw.set_xlim([x_min, x_max])

    ax_pitch.set_ylabel('Pitch (degrees)')
    ax_roll.set_ylabel('Roll (degrees)')
    ax_yaw.set_ylabel('Yaw (degrees)')
    ax_yaw.set_xlabel('Time (s)')

    ax_pitch.axhline(0, color='gray', linewidth=0.5)
    ax_roll.axhline(0, color='gray', linewidth=0.5)
    ax_yaw.axhline(0, color='gray', linewidth=0.5)

    ax_pitch.legend()
    ax_roll.legend()
    ax_yaw.legend()

    return plane_body, info_text, ax_pitch, ax_roll, ax_yaw


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
        #data['yaw'].append(message['yaw'])
        data['yaw'].append(0.0)
        data['altitude'].append(message['altitude'])
        data['motor1'].append(message['motor1'])
        data['motor2'].append(message['motor2'])
        data['motor3'].append(message['motor3'])
        data['motor4'].append(message['motor4'])
        data['pidpitch'].append(message['pidpitch'])
        data['pidroll'].append(message['pidroll'])
        data['pidyaw'].append(message['pidyaw'])
        data['pidalt'].append(message['pidalt'])  
        if len(data['time']) == 0:
            data['time'].append(0)
        else:
            data['time'].append(data['time'][-1] + 1)
    except json.JSONDecodeError as exc:
        print("Failed to decode JSON message", exc)
    except KeyError as e:
        print(f"Missing key in message: {e}")

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
ani = FuncAnimation(fig, update, blit=False, interval=100, cache_frame_data=False)
command_thread = Thread(target=send_command, args=(client,), daemon=True)
command_thread.start()
plt.show()
