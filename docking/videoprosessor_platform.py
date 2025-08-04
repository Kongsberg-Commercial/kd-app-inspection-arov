#!/usr/bin/env python3
"""
Video processing node for platform docking, with interactive tuning controls.
Now uses horizontal guide lines: blue = desired dock line, green = detected dock line.
"""

import os
import sys
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

# Sanity check
if USE_FILE and not os.path.isfile(VIDEO_FILE):
    raise FileNotFoundError(f"Test video not found at: {VIDEO_FILE!r}")

# GStreamer imports if needed
if not USE_FILE:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    import socket

# --- Video wrapper --------------------------------------------------------

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
        buf    = sample.get_buffer()
        caps   = sample.get_caps().get_structure(0)
        h, w   = caps.get_value("height"), caps.get_value("width")
        arr    = np.ndarray((h, w, 3),
                            buffer=buf.extract_dup(0, buf.get_size()),
                            dtype=np.uint8)
        self._new_frame = arr
        return Gst.FlowReturn.OK

    def frame_available(self):
        return (self.cap.isOpened() if USE_FILE else self._new_frame is not None)

    def frame(self):
        if USE_FILE:
            ret, frm = self.cap.read()
            return frm if ret else None
        else:
            frm = self._new_frame
            self._new_frame = None
            return frm

# --- Detection helper -----------------------------------------------------

def detect_platform(mask, min_area):
    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    large = max(cnts, key=cv.contourArea)
    if cv.contourArea(large) < min_area:
        return None
    (cx, cy), (w, h), ang = cv.minAreaRect(large)
    if w < h:
        ang += 90
    return {"cx": int(cx), "cy": int(cy), "angle": ang}

def nothing(x):
    pass

# --- Main -----------------------------------------------------------------

def main():
    video = Video()
    mqttc = mqtt.Client()
    mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    mqttc.connect(BROKER_IP, BROKER_PORT)
    mqttc.loop_start()
    print(f"▶️  Started, publishing to {PLATFORM_TOPIC}")

    # Create tuning controls
    cv.namedWindow("Controls", cv.WINDOW_NORMAL)
    cv.resizeWindow("Controls", 400, 240)
    cv.createTrackbar("Canny Lo",    "Controls",  50,   255, nothing)
    cv.createTrackbar("Canny Hi",    "Controls", 150,   500, nothing)
    cv.createTrackbar("Blur K",      "Controls",   5,    25, nothing)   # odd
    cv.createTrackbar("Morph K",     "Controls",   7,    31, nothing)   # odd
    cv.createTrackbar("Min Area",    "Controls",1000, 20000, nothing)
    cv.createTrackbar("Guide Offs %","Controls",  50,    100, nothing)  # percent down

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

        # Read tuning values
        lo = cv.getTrackbarPos("Canny Lo",    "Controls")
        hi = cv.getTrackbarPos("Canny Hi",    "Controls")
        bk = cv.getTrackbarPos("Blur K",      "Controls") | 1
        mk = cv.getTrackbarPos("Morph K",     "Controls") | 1
        ma = cv.getTrackbarPos("Min Area",    "Controls")
        gp = cv.getTrackbarPos("Guide Offs %","Controls")  # 0-100%

        # Compute horizontal guide position
        guide_y = int((gp / 100.0) * h)

        # Build mask
        gray    = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (bk, bk), 0)
        edges   = cv.Canny(blurred, lo, hi)
        kernel  = cv.getStructuringElement(cv.MORPH_RECT, (mk, mk))
        mask    = cv.morphologyEx(edges, cv.MORPH_CLOSE, kernel)

        # Detect baseline
        plat = detect_platform(mask, ma)

        # Annotate
        annotated = img.copy()
        # Draw blue horizontal guide
        cv.line(annotated, (0, guide_y), (w, guide_y), (255, 0, 0), 2)

        if plat:
            cy  = plat["cy"]
            ang = plat["angle"]
            y_err = cy - guide_y

            # Draw green horizontal at detected baseline
            cv.line(annotated, (0, cy), (w, cy), (0, 255, 0), 2)
            # Overlay text
            cv.putText(annotated,
                       f"y_err={y_err} px  θ={ang:.1f}°",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
            # Publish
            mqttc.publish(PLATFORM_TOPIC, json.dumps({
                "offset_pixels": y_err,
                "angle_deg":     round(ang,2),
                "timestamp":     time.time()
            }))
        else:
            cv.putText(annotated,
                       "No platform",
                       (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)

        # Show
        cv.imshow("Raw Frame",    img)
        cv.imshow("Mask Debug",   mask)
        cv.imshow("Detection",    annotated)

        if cv.waitKey(30) & 0xFF == ord('q'):
            break

    cv.destroyAllWindows()
    if USE_FILE:
        video.cap.release()

if __name__ == "__main__":
    main()
