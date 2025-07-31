import json
import threading
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from pyproj import Transformer
from pymavlink import mavutil  # Added for MAVLink integration
import paho.mqtt.client as mqtt

# --- Configuration ---
REMOTE_BROKER_HOST = "100.78.45.94"
REMOTE_BROKER_PORT = 1883
MQTT_USERNAME = "formula2boat"
MQTT_PASSWORD = "formula2boat"
TOPIC_DRONE = "sommer/rovposition"
TOPIC_USV = "sommer/position"
UTM_EPSG = 32632
TRAIL_LENGTH = 100

# Transformer from lat/lon to UTM coordinates
transformer = Transformer.from_crs("epsg:4326", f"epsg:{UTM_EPSG}", always_xy=True)

# MAVLink connection (adjust connection string as needed)
mav = mavutil.mavlink_connection('udp:0.0.0.0:14550')

# Thread-safe storage
drone_trail = deque(maxlen=TRAIL_LENGTH)
usv_trail = deque(maxlen=TRAIL_LENGTH)
latest_drone = None
latest_usv = None
lock = threading.Lock()

# Global arrows
arrow_usv = None
arrow_drone = None

# Function to read drone heading from MAVLink
def read_mav_heading():
    msg = mav.recv_match(type='ATTITUDE', blocking=False)
    if msg and hasattr(msg, 'yaw'):
        # msg.yaw is in radians, convert to degrees
        return float(np.degrees(msg.yaw))
    return None


def on_message(client, userdata, msg):
    global latest_drone, latest_usv
    try:
        data = json.loads(msg.payload.decode('utf-8', errors='ignore'))
    except json.JSONDecodeError:
        return

    # Parse drone data
    if msg.topic == TOPIC_DRONE:
        lat = data.get('lat')
        lon = data.get('lng')
        alt = -data.get('down')
        if None in (lat, lon, alt):
            return
        x, y = transformer.transform(lon, lat)
        heading = data.get('heading') or read_mav_heading()  # use JSON or MAVLink
        with lock:
            latest_drone = (x, y, float(alt), float(heading) if heading is not None else None)
            drone_trail.append((x, y, float(alt)))

    # Parse USV data
    elif msg.topic == TOPIC_USV:
        lat = data.get('lat')
        lon = data.get('lng')
        hd = data.get('heading')
        if None in (lat, lon, hd):
            return
        x, y = transformer.transform(lon, lat)
        with lock:
            latest_usv = (x, y, 0.0, float(hd))
            usv_trail.append((x, y, 0.0))

# Set up MQTT client
client = mqtt.Client()
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.on_message = on_message
client.on_connect = lambda c, u, f, rc: c.subscribe([(TOPIC_DRONE, 0), (TOPIC_USV, 0)])
client.connect(REMOTE_BROKER_HOST, REMOTE_BROKER_PORT)
threading.Thread(target=client.loop_forever, daemon=True).start()

# Set up 3D plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot elements for trails and markers
drone_line, = ax.plot([], [], [], '-o', color='red', alpha=0.6, linewidth=2, markersize=4, label='Drone Trail')
usv_line, = ax.plot([], [], [], '-o', color='blue', alpha=0.6, linewidth=2, markersize=4, label='USV Trail')

drone_scatter = ax.scatter([], [], [], color='red', s=120, marker='^', edgecolors='black', label='Drone')
usv_scatter = ax.scatter([], [], [], color='blue', s=120, marker='s', edgecolors='black', label='USV')

# Add a translucent sea surface at z = 0
sea_extent = 200  # adjust as needed
xx = np.linspace(-sea_extent, sea_extent, 2)
yy = np.linspace(-sea_extent, sea_extent, 2)
xx, yy = np.meshgrid(xx, yy)
zz = np.zeros_like(xx)
sea_surface = ax.plot_surface(xx, yy, zz, color='cyan', alpha=0.3)

ax.legend(loc='upper left')
ax.set_xlabel('Easting (m)')
ax.set_ylabel('Northing (m)')
ax.set_zlabel('Altitude (m)')
ax.grid(True)

# Update function for animation
def update(frame):
    global arrow_usv, arrow_drone
    with lock:
        drone = latest_drone
        usv = latest_usv
        trail_d = list(drone_trail)
        trail_u = list(usv_trail)

    # Update trails
    if trail_d:
        xd, yd, zd = zip(*trail_d)
        drone_line.set_data(xd, yd)
        drone_line.set_3d_properties(zd)
    if trail_u:
        xu, yu, zu = zip(*trail_u)
        usv_line.set_data(xu, yu)
        usv_line.set_3d_properties(zu)

    # Update latest positions
    if drone:
        x_d, y_d, z_d, h_d = drone
        drone_scatter._offsets3d = ([x_d], [y_d], [z_d])
    if usv:
        x_u, y_u, z_u, h_u = usv
        usv_scatter._offsets3d = ([x_u], [y_u], [z_u])

    # Adjust axes limits dynamically
    all_pts = np.array(trail_d + trail_u) if trail_d or trail_u else np.empty((0,3))
    if all_pts.size:
        m = 5
        ax.set_xlim(all_pts[:,0].min()-m, all_pts[:,0].max()+m)
        ax.set_ylim(all_pts[:,1].min()-m, all_pts[:,1].max()+m)
        ax.set_zlim(-20, 5)

    # Remove previous arrows
    if arrow_usv:
        arrow_usv.remove()
    if arrow_drone:
        arrow_drone.remove()

    # Draw USV heading arrow
    arrow_len = 5
    if usv:
        r_u = np.deg2rad(h_u)
        arrow_usv = ax.quiver(x_u, y_u, z_u, np.cos(r_u), np.sin(r_u), 0,
                              length=arrow_len, color='blue', normalize=True)

    # Draw Drone heading arrow
    if drone and h_d is not None:
        r_d = np.deg2rad(h_d)
        arrow_drone = ax.quiver(x_d, y_d, z_d, np.cos(r_d), np.sin(r_d), 0,
                                length=arrow_len, color='red', normalize=True)

    ax.set_title('Real-Time Drone and USV Tracking with Sea Surface', fontsize=16)

# Run animation
ani = FuncAnimation(fig, update, interval=500)
plt.tight_layout()
plt.show()
