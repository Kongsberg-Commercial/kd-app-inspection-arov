# autonomous_docking.py (Revised)
import time
import math
import json
import threading
from enum import Enum
import paho.mqtt.client as mqtt
from pymavlink import mavutil
from geopy.distance import geodesic
from geopy.point import Point

# --- Configuration (UPDATED) ---
# MAVLink connection
MAVLINK_CONNECTION_STR = 'udpin:0.0.0.0:14550'

# MQTT Broker Configuration
MQTT_BROKER_HOST = "100.78.45.94"
MQTT_BROKER_PORT = 1883
MQTT_USERNAME = "formula2boat"
MQTT_PASSWORD = "formula2boat"

# MQTT Topics
MQTT_TOPIC_ROV_GPS = "sommer/rovposition"
MQTT_TOPIC_BOAT_GPS = "sommer/position"
MQTT_TOPIC_LINE_DETECTION = "/docking/line"      # From docking_videoprocessor.py
MQTT_TOPIC_PLATFORM_CMD = "/rov_deployment"      # Topic to control the platform

# Docking Parameters
PRE_DOCK_DISTANCE_M = 5.0
PRE_DOCK_DEPTH_M = -2.0
FINAL_DOCK_DEPTH_M = -0.1
VISUAL_DOCK_FWD_SPEED = 200
DOCKING_DURATION_S = 8.0

# --- State Machine ---
class DockingState(Enum):
    STARTING = 1
    NAV_TO_WAYPOINT = 2
    SETTING_FINAL_DEPTH = 3
    VISUAL_DOCKING = 4
    DOCKED = 5
    COMPLETE = 6
    FAIL = 7

# --- Main Docking Class ---
class AutonomousDocker:
    def __init__(self):
        self.state = DockingState.STARTING
        # Use a more descriptive naming scheme
        self.rov_data = {"lat": 0, "lon": 0, "depth": 0, "heading": 0}
        self.usv_data = {"lat": 0, "lon": 0, "heading": 0}
        self.line_detection = {"distance": 0, "angle": 0, "detected": False}
        self.target_waypoint = None
        self.visual_dock_start_time = None

        self.master = mavutil.mavlink_connection(MAVLINK_CONNECTION_STR)
        self.master.wait_heartbeat()
        print("✅ MAVLink Connected")

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT)
        self.mqtt_client.subscribe([(MQTT_TOPIC_ROV_GPS, 0),
                                    (MQTT_TOPIC_BOAT_GPS, 0),
                                    (MQTT_TOPIC_LINE_DETECTION, 0)])
        threading.Thread(target=self.mqtt_client.loop_forever, daemon=True).start()
        print("✅ MQTT Connected")

        # PID Controllers
        self.pd_lat = {'kp': 10, 'kd': 8}
        self.pd_yaw = {'kp': 12, 'kd': 5}
        self.prev_lat_error = 0
        self.prev_yaw_error = 0
        
        self.pid_heading = {'kp': 300, 'ki': 5, 'kd': 20}
        self.heading_integral = 0
        self.prev_time = time.time()

    # --- on_message function UPDATED ---
    def on_message(self, client, userdata, msg):
        """Handles incoming MQTT messages."""
        try:
            payload = json.loads(msg.payload.decode('utf-8', errors='ignore'))
            
            if msg.topic == MQTT_TOPIC_ROV_GPS:
                # Parses data from sommer/rovposition
                lat = payload.get('lat')
                lon = payload.get('lng') # Note: key is 'lng'
                depth = -payload.get('down', 0) # Note: key is 'down'
                heading = payload.get('heading', 0)
                if lat is not None and lon is not None:
                    self.rov_data.update({"lat": lat, "lon": lon, "depth": depth, "heading": heading})

            elif msg.topic == MQTT_TOPIC_BOAT_GPS:
                # Parses data from sommer/position
                lat = payload.get('lat')
                lon = payload.get('lng') # Note: key is 'lng'
                heading = payload.get('heading')
                if lat is not None and lon is not None and heading is not None:
                    self.usv_data.update({"lat": lat, "lon": lon, "heading": heading})

            elif msg.topic == MQTT_TOPIC_LINE_DETECTION:
                self.line_detection.update(payload)
                self.line_detection['detected'] = True

        except (json.JSONDecodeError, KeyError) as e:
            print(f"MQTT Error parsing topic {msg.topic}: {e}")

    def arm(self):
        print("Arming vehicle...")
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        self.master.motors_armed_wait()
        print("✅ Armed!")

    def disarm(self):
        print("Disarming vehicle...")
        self.master.mav.command_long_send(self.master.target_system, self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        self.master.motors_disarmed_wait()
        print("✅ Disarmed!")

    def set_mode(self, mode):
        mode_id = self.master.mode_mapping().get(mode)
        if mode_id is None: return
        self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        print(f"Mode set to {mode}")
        time.sleep(1)

    def set_target_depth(self, depth):
        self.master.mav.set_position_target_global_int_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b110111111000, 0, 0, depth, 0, 0, 0, 0, 0, 0, 0, 0)
        print(f"Setting target depth to {depth}m")

    def manual_control(self, x=0, y=0, z=500, r=0):
        self.master.mav.manual_control_send(self.master.target_system, x, y, z, r, 0)

    def calculate_target_waypoint(self):
        if not self.usv_data["lat"] or not self.usv_data["lon"]: return None
        start_point = Point(self.usv_data["lat"], self.usv_data["lon"])
        bearing = (self.usv_data["heading"] + 180) % 360
        return geodesic(meters=PRE_DOCK_DISTANCE_M).destination(start_point, bearing)

    def calculate_bearing_and_distance(self):
        if not self.target_waypoint or not self.rov_data["lat"]: return None, None
        p1 = (self.rov_data["lat"], self.rov_data["lon"])
        p2 = (self.target_waypoint.latitude, self.target_waypoint.longitude)
        dist = geodesic(p1, p2).meters
        lat1, lon1, lat2, lon2 = map(math.radians, [p1[0], p1[1], p2[0], p2[1]])
        dLon = lon2 - lon1
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        return bearing, dist

    def run(self):
        self.arm()
        self.set_mode('MANUAL')

        while self.state not in [DockingState.COMPLETE, DockingState.FAIL]:
            dt = time.time() - self.prev_time
            if dt == 0: continue
            self.prev_time = time.time()
            
            if self.state == DockingState.STARTING:
                print("State: STARTING. Waiting for valid GPS data...")
                if self.usv_data["lat"] != 0 and self.rov_data["lat"] != 0:
                    self.set_target_depth(PRE_DOCK_DEPTH_M)
                    self.state = DockingState.NAV_TO_WAYPOINT
            
            elif self.state == DockingState.NAV_TO_WAYPOINT:
                self.target_waypoint = self.calculate_target_waypoint()
                bearing, dist = self.calculate_bearing_and_distance()
                if bearing is None: continue
                
                print(f"State: NAV_TO_WAYPOINT. Dist: {dist:.2f}m, USV Heading: {self.usv_data['heading']:.1f}°")
                heading_err = self.usv_data["heading"] - self.rov_data.get("heading", self.usv_data["heading"])
                heading_err = (heading_err + 180) % 360 - 180
                self.heading_integral += heading_err * dt
                self.heading_integral = max(min(self.heading_integral, 10), -10)
                r_out = self.pid_heading['kp'] * heading_err + self.pid_heading['ki'] * self.heading_integral
                
                bearing_err = (bearing - self.rov_data["heading"] + 180) % 360 - 180
                x_out = 400 if abs(bearing_err) < 20 and dist > 1.5 else 100
                self.manual_control(x=int(x_out), y=0, z=500, r=int(max(min(r_out, 400), -400)))
                
                if dist < 1.0 and abs(heading_err) < 5:
                    self.set_target_depth(FINAL_DOCK_DEPTH_M)
                    self.state = DockingState.SETTING_FINAL_DEPTH

            elif self.state == DockingState.SETTING_FINAL_DEPTH:
                print(f"State: SETTING_FINAL_DEPTH. Current Depth: {self.rov_data['depth']:.2f}m")
                self.manual_control()
                if abs(self.rov_data['depth'] - FINAL_DOCK_DEPTH_M) < 0.1:
                    self.state = DockingState.VISUAL_DOCKING
                    
            elif self.state == DockingState.VISUAL_DOCKING:
                if not self.line_detection['detected']:
                    print("State: VISUAL_DOCKING. Waiting for line detection...")
                    self.manual_control()
                    continue
                
                lat_error, yaw_error = self.line_detection['distance'], self.line_detection['angle']
                lat_deriv, yaw_deriv = (lat_error - self.prev_lat_error)/dt, (yaw_error - self.prev_yaw_error)/dt
                y_out = self.pd_lat['kp'] * lat_error + self.pd_lat['kd'] * lat_deriv
                r_out = self.pd_yaw['kp'] * yaw_error + self.pd_yaw['kd'] * yaw_deriv
                self.prev_lat_error, self.prev_yaw_error = lat_error, yaw_error
                
                x_out = 0
                if abs(lat_error) < 25 and abs(yaw_error) < 5:
                    x_out = VISUAL_DOCK_FWD_SPEED
                    if self.visual_dock_start_time is None: self.visual_dock_start_time = time.time()
                else:
                    self.visual_dock_start_time = None

                print(f"State: VISUAL_DOCKING. LatErr: {lat_error}px, YawErr: {yaw_error}° | Out Y:{int(y_out)}, R:{int(r_out)}")
                self.manual_control(x=x_out, y=int(-y_out), z=500, r=int(-r_out))

                if self.visual_dock_start_time and (time.time() - self.visual_dock_start_time > DOCKING_DURATION_S):
                    self.state = DockingState.DOCKED

            # --- DOCKED state UPDATED ---
            elif self.state == DockingState.DOCKED:
                print("State: DOCKED. Sending command 'retrieve' to raise platform.")
                self.manual_control() # Stop all motion
                # Publish the 'retrieve' command to the correct topic
                self.mqtt_client.publish(MQTT_TOPIC_PLATFORM_CMD, 'retrieve', qos=1)
                time.sleep(2)
                self.disarm()
                self.state = DockingState.COMPLETE

            time.sleep(0.1)

        print("Docking sequence finished.")
        self.mqtt_client.disconnect()

if __name__ == "__main__":
    docker = AutonomousDocker()
    try:
        docker.run()
    except KeyboardInterrupt:
        print("Interrupt received. Disarming...")
        docker.manual_control()
        docker.disarm()
    except Exception as e:
        print(f"An unhandled exception occurred: {e}")
        docker.manual_control()
        docker.disarm()