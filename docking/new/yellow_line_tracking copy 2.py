#!/usr/bin/env python3
"""
Yellow line tracking script for close-range platform docking.
Detects a yellow marker line, calculates center offset and angle,
and publishes to MQTT. Includes HSV mask tuning sliders and ROI filtering.
"""

import cv2 as cv
import numpy as np
import time
import json
import os
import paho.mqtt.client as mqtt

# === Configuration ===
USE_FILE = True
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
VIDEO_FILE = os.path.join(BASE_DIR, "platform2.mkv")
VIDEO_PORT = 5601

BROKER_IP = "100.86.23.108"
BROKER_PORT = 1883
MQTT_TOPIC = "bluerov2/center_line"
MQTT_USERNAME = "formula2boat"
MQTT_PASSWORD = "formula2boat"

# === Video Source ===
class Video:
    def __init__(self, port=VIDEO_PORT):
        if USE_FILE:
            self.cap = cv.VideoCapture(VIDEO_FILE)
        else:
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst
            Gst.init(None)
            self._new_frame = None
            cmd = (
                f"udpsrc port={port} ! application/x-rtp,payload=96 ! "
                "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
                "video/x-raw,format=BGR ! appsink emit-signals=true sync=false max-buffers=2 drop=true"
            )
            self.pipe = Gst.parse_launch(cmd)
            self.pipe.set_state(Gst.State.PLAYING)
            self.sink = self.pipe.get_by_name("appsink0")
            self.sink.connect("new-sample", self._on_sample)

    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        h, w = caps.get_value("height"), caps.get_value("width")
        arr = np.ndarray((h, w, 3),
                         buffer=buf.extract_dup(0, buf.get_size()),
                         dtype=np.uint8)
        self._new_frame = arr
        return Gst.FlowReturn.OK

    def frame_available(self):
        if USE_FILE:
            return self.cap.isOpened()
        else:
            return self._new_frame is not None

    def frame(self):
        if USE_FILE:
            ret, frm = self.cap.read()
            return frm if ret else None
        else:
            frm = self._new_frame
            self._new_frame = None
            return frm

# === Controls ===
def nothing(x): pass

def setup_controls():
    cv.namedWindow("Controls", cv.WINDOW_NORMAL)
    cv.resizeWindow("Controls", 400, 300)
    cv.createTrackbar("Hue Lo", "Controls", 20, 179, nothing)
    cv.createTrackbar("Hue Hi", "Controls", 35, 179, nothing)
    cv.createTrackbar("Sat Lo", "Controls", 40, 255, nothing)
    cv.createTrackbar("Sat Hi", "Controls", 203, 255, nothing)
    cv.createTrackbar("Val Lo", "Controls", 0, 255, nothing)
    cv.createTrackbar("Val Hi", "Controls", 255, 255, nothing)

# === Main ===
def main():
    if USE_FILE and not os.path.isfile(VIDEO_FILE):
        print(f"❌ Video file not found: {VIDEO_FILE}")
        return

    video = Video()
    mqttc = mqtt.Client()
    mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    mqttc.connect(BROKER_IP, BROKER_PORT)
    mqttc.loop_start()

    setup_controls()
    print(f"📹 Tracking yellow line and publishing to {MQTT_TOPIC}")

    while True:
        if not video.frame_available():
            time.sleep(0.01)
            continue

        img = video.frame()
        if img is None:
            break

        h, w = img.shape[:2]
        center_x = w // 2

        # --- HSV Range ---
        h_lo = cv.getTrackbarPos("Hue Lo", "Controls")
        h_hi = cv.getTrackbarPos("Hue Hi", "Controls")
        s_lo = cv.getTrackbarPos("Sat Lo", "Controls")
        s_hi = cv.getTrackbarPos("Sat Hi", "Controls")
        v_lo = cv.getTrackbarPos("Val Lo", "Controls")
        v_hi = cv.getTrackbarPos("Val Hi", "Controls")

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower_yellow = np.array([h_lo, s_lo, v_lo])
        upper_yellow = np.array([h_hi, s_hi, v_hi])
        mask = cv.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # --- ROI Masking (10%–90% vertical area only) ---
        roi_mask = np.zeros_like(mask)
        y_min = int(h * 0.1)
        y_max = int(h * 0.9)
        roi_mask[y_min:y_max, :] = 255
        masked = cv.bitwise_and(mask, roi_mask)

        # --- Line Detection ---
        lines = cv.HoughLinesP(masked, 1, np.pi/180, threshold=60,
                               minLineLength=50, maxLineGap=15)

        annotated = img.copy()
        cv.line(annotated, (center_x, 0), (center_x, h), (255, 0, 0), 2)
        cv.rectangle(annotated, (0, y_min), (w, y_max), (0, 0, 255), 2)

        if lines is not None:
            # Filter by line center y inside ROI
            def line_center_y(l): return (l[0][1] + l[0][3]) // 2
            filtered = [l for l in lines if y_min <= line_center_y(l) <= y_max]
            if not filtered:
                filtered = lines

            # Pick the longest vertical-ish line
            best_line = max(filtered, key=lambda l: abs(l[0][1] - l[0][3]))
            x1, y1, x2, y2 = best_line[0]
            cv.line(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)

            angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
            cx = (x1 + x2) // 2
            x_offset = cx - center_x

            msg = {
                "distance": int(x_offset),
                "angle": round(angle, 2),
                "timestamp": time.time()
            }
            mqttc.publish(MQTT_TOPIC, json.dumps(msg))

            cv.putText(annotated, f"x={x_offset} angle={angle:.1f}°",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            cv.putText(annotated, "No yellow line detected",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # --- Show Windows ---
        cv.imshow("Raw", img)
        cv.imshow("Mask", mask)
        cv.imshow("ROI Masked", masked)
        cv.imshow("Detection", annotated)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cv.destroyAllWindows()
    if USE_FILE:
        video.cap.release()

if __name__ == "__main__":
    main()
