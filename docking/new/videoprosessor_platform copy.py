#!/usr/bin/env python3
"""
Video processing node for platform docking, with visual debug and
a green orientation line drawn through the detected platform.
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
MIN_AREA       = 1000

# Sanity check
if USE_FILE and not os.path.isfile(VIDEO_FILE):
    raise FileNotFoundError(f"Test video not found at: {VIDEO_FILE!r}")

# GStreamer imports if needed
if not USE_FILE:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    import socket

class Video:
    """Wraps either a file-based or UDP-GStreamer capture."""
    def __init__(self, port=VIDEO_PORT):
        self._new_frame = None
        if USE_FILE:
            self.cap = cv.VideoCapture(VIDEO_FILE)
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open video: {VIDEO_FILE!r}")
        else:
            Gst.init(None)
            self.video_source   = f"udpsrc port={port}"
            self.video_codec    = (
                "! application/x-rtp, payload=96 "
                "! rtph264depay ! h264parse ! avdec_h264"
            )
            self.video_decode   = (
                "! decodebin ! videoconvert "
                "! video/x-raw,format=(string)BGR"
            )
            self.video_sink_conf = (
                "! appsink emit-signals=true sync=false "
                "max-buffers=2 drop=true"
            )
            cmd = " ".join([
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])
            self.pipe = Gst.parse_launch(cmd)
            self.pipe.set_state(Gst.State.PLAYING)
            self.sink = self.pipe.get_by_name("appsink0")
            self.sink.connect("new-sample", self._on_sample)

    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        caps = sample.get_caps().get_structure(0)
        h = caps.get_value("height")
        w = caps.get_value("width")
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

def main():
    video = Video()
    mqttc = mqtt.Client()
    mqttc.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    mqttc.connect(BROKER_IP, BROKER_PORT)
    mqttc.loop_start()
    print(f"▶️  Started, publishing to {PLATFORM_TOPIC}")

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

        # 1) Build mask: blur → Canny → close
        gray    = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (5, 5), 0)
        edges   = cv.Canny(blurred, 50, 150)
        kernel  = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
        mask    = cv.morphologyEx(edges, cv.MORPH_CLOSE, kernel)

        # 2) Detect platform
        plat = detect_platform(mask, MIN_AREA)

        # 3) Annotate
        annotated = img.copy()
        # draw vertical center guide
        cv.line(annotated, (center_x, 0), (center_x, h), (255, 0, 0), 2)

        if plat:
            cx, cy = plat["cx"], plat["cy"]
            ang     = plat["angle"]
            x_err   = cx - center_x

            # draw centroid
            cv.circle(annotated, (cx, cy), 5, (0, 255, 0), -1)

            # draw orientation line
            theta = np.deg2rad(ang)
            dx, dy = np.cos(theta), np.sin(theta)
            length = max(w, h)
            pt1 = (int(cx - dx*length), int(cy - dy*length))
            pt2 = (int(cx + dx*length), int(cy + dy*length))
            cv.line(annotated, pt1, pt2, (0, 255, 0), 2, cv.LINE_AA)

            # publish
            msg = {"distance": x_err, "angle": round(ang,2), "timestamp": time.time()}
            mqttc.publish(PLATFORM_TOPIC, json.dumps(msg))

            # overlay text
            cv.putText(annotated,
                       f"d={x_err} θ={ang:.1f}°",
                       (10, 30),
                       cv.FONT_HERSHEY_SIMPLEX,
                       1.0, (0,255,0), 2)
        else:
            cv.putText(annotated, "No platform",
                       (10, 30),
                       cv.FONT_HERSHEY_SIMPLEX,
                       1.0, (0,0,255), 2)

        # 4) Show windows
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
