# import paho.mqtt.client as mqtt
# import json

# # Datos de conexión al broker MQTT
# BROKER_URL = "localhost"  # O la IP de tu servidor Mosquitto si está en otra máquina
# BROKER_PORT = 1883
# TOPIC = "drone/angles"

# # Función callback cuando el cliente se conecta al broker
# def on_connect(client, userdata, flags, rc):
#     print(f"Connected with result code {rc}")
#     # Suscribirse al tema cuando se conecta
#     client.subscribe(TOPIC)

# # Función callback cuando se recibe un mensaje
# def on_message(client, userdata, msg):
#     print(f"Topic: {msg.topic}")
#     payload = msg.payload.decode()
#     print(f"Message: {payload}")
    
#     # Procesar el mensaje JSON
#     try:
#         data = json.loads(payload)
#         pitch = data.get('pitch')
#         roll = data.get('roll')
#         yaw = data.get('yaw')
#         print(f"Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
#     except json.JSONDecodeError:
#         print("Failed to decode JSON message")

# # Crear una instancia de cliente MQTT
# client = mqtt.Client()

# # Asignar funciones callback
# client.on_connect = on_connect
# client.on_message = on_message

# # Conectarse al broker MQTT
# client.connect(BROKER_URL, BROKER_PORT, 60)

# # Mantener el cliente en funcionamiento para recibir mensajes
# client.loop_forever()

import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import json
from collections import deque

# Configuration
BROKER_URL = "localhost"  # Update with your MQTT broker address
BROKER_PORT = 1883
TOPIC = "drone/angles"

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

# Set up MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_URL, BROKER_PORT, 60)
client.loop_start()

# Set up animation
ani = FuncAnimation(fig, update, blit=True, interval=100, cache_frame_data=False)

plt.show()



