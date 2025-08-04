#!/usr/bin/env python3
"""
Video processing node for platform docking, focusing on detecting platform bottom edge.
Publishes x-offset and line angle to MQTT.
"""

import os
import time
import json
import numpy as np
import paho.mqtt.client as mqtt
import cv2 as cv

# === Configuration ===
USE_FILE       = True
BASE_DIR       = os.path.dirname(os.path.abspath(__file__))
VIDEO_FILE     = os.path.join(BASE_DIR, "platform2.mkv")

BROKER_IP      = "100.86.23.108"
BROKER_PORT    = 1883
MQTT_USERNAME  = "formula2boat"
MQTT_PASSWORD  = "formula2boat"
PLATFORM_TOPIC = "bluerov2/platform"

VIDEO_PORT     = 5601

if USE_FILE and not os.path.isfile(VIDEO_FILE):
    raise FileNotFoundError(f"Test video not found at: {VIDEO_FILE!r}")

# GStreamer for live stream
if not USE_FILE:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    import socket

# --- Video Wrapper --------------------------------------------------------
class Video:
    def __init__(self, port=VIDEO_PORT):
        self._new_frame = None
        if USE_FILE:
            self.cap = cv.VideoCapture(VIDEO_FILE)
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open video: {VIDEO_FILE!r}")
        else:
            Gst.init(None)
            cmd = (
                f"udpsrc port={port} ! "
                "application/x-rtp,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! "
                "videoconvert ! video/x-raw,format=BGR ! appsink emit-signals=true sync=false max-buffers=2 drop=true"
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

def nothing(x):
    pass

# --- Main -----------------------------------------------------------------
def main():
    video = Video()

    mqttc = mqtt.Client()
    mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    mqttc.connect(BROKER_IP, BROKER_PORT)
    mqttc.loop_start()

    print(f"▶️ Started, publishing to {PLATFORM_TOPIC}")

    # Create tuning controls
    cv.namedWindow("Controls", cv.WINDOW_NORMAL)
    cv.resizeWindow("Controls", 400, 200)
    cv.createTrackbar("Canny Lo",   "Controls", 50, 255, nothing)
    cv.createTrackbar("Canny Hi",   "Controls", 150, 500, nothing)
    cv.createTrackbar("Blur K",     "Controls", 5, 25, nothing)  # odd
    cv.createTrackbar("Morph K",    "Controls", 7, 31, nothing)  # odd

    center_x = None

    while True:
        if not video.frame_available():
            if USE_FILE:
                print("🔚 End of video file.")
                break
            continue

        img = video.frame()
        if img is None:
            continue

        h, w = img.shape[:2]
        if center_x is None:
            center_x = w // 2

        # --- Read Tuning Values ---
        lo = cv.getTrackbarPos("Canny Lo", "Controls")
        hi = cv.getTrackbarPos("Canny Hi", "Controls")
        bk = cv.getTrackbarPos("Blur K",   "Controls") | 1
        mk = cv.getTrackbarPos("Morph K",  "Controls") | 1

        # --- Edge Detection ---
        gray    = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (bk, bk), 0)
        edges   = cv.Canny(blurred, lo, hi)
        kernel  = cv.getStructuringElement(cv.MORPH_RECT, (mk, mk))
        mask    = cv.morphologyEx(edges, cv.MORPH_CLOSE, kernel)

        # --- Focus on Middle-Bottom ROI (avoid bottom)
        roi_mask = np.zeros_like(mask)
        roi_mask[int(h * 0.1):int(h * 0.90), :] = 255  # from 10% to 95% of height
        masked_edges = cv.bitwise_and(mask, roi_mask)

        # --- Hough Line Detection ---
        lines = cv.HoughLinesP(masked_edges, 1, np.pi/180, threshold=80,
                               minLineLength=100, maxLineGap=20)

        # --- Annotate ---
        annotated = img.copy()
        cv.line(annotated, (center_x, 0), (center_x, h), (255, 0, 0), 2)

        if lines is not None:
            target_min_y = int(h * 0.1)
            target_max_y = int(h * 0.90)

            def line_angle(line):
                x1, y1, x2, y2 = line[0]
                return np.arctan2(y2 - y1, x2 - x1)

            def line_center_y(line):
                x1, y1, x2, y2 = line[0]
                return (y1 + y2) // 2

            # Filter lines in vertical ROI
            filtered_lines = [l for l in lines if target_min_y <= line_center_y(l) <= target_max_y]
            if not filtered_lines:
                filtered_lines = lines  # fallback

            best_line = min(filtered_lines, key=lambda l: abs(np.degrees(line_angle(l))))
            x1, y1, x2, y2 = best_line[0]
            cv.line(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Compute line angle
            angle_rad = np.arctan2(y2 - y1, x2 - x1)
            angle_deg = np.degrees(angle_rad)

            # Distance error (center offset)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            x_err = cx - center_x

            # Draw guidance arrow
            cv.arrowedLine(annotated, (center_x, h), (cx, cy), (0, 255, 255), 3)

            # Publish data
            msg = {
                "distance": int(x_err),
                "angle": float(round(angle_deg, 2)),
                "timestamp": float(time.time())
            }
            mqttc.publish(PLATFORM_TOPIC, json.dumps(msg))

            cv.putText(annotated, f"x_err={x_err} angle={angle_deg:.1f}°",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # Optional: visualize ROI
            cv.rectangle(annotated, (0, target_min_y), (w, target_max_y), (0, 0, 255), 2)
        else:
            cv.putText(annotated, "No platform edge found",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # --- Show Windows ---
        cv.imshow("Raw Frame", img)
        cv.imshow("Mask Debug", mask)
        cv.imshow("Detection", annotated)

        key = cv.waitKey(30) & 0xFF
        if key == ord('q'):
            break

    cv.destroyAllWindows()
    if USE_FILE:
        video.cap.release()

if __name__ == "__main__":
    main()
