# docking_videoprocessor_file.py
# A test version of the docking video processor that reads from a local video file,
# visualizes detection results in real-time windows (resizable), and supports configurable HSV thresholds.
# Now includes robust handling when no line is detected.

import sys
import math
import cv2 as cv
import numpy as np
import time
import threading
import paho.mqtt.client as mqtt
import json
import argparse

# --- Video class for file input ---
class VideoFile:
    """
    VideoFile class uses cv2.VideoCapture to read from a local video file.
    """
    def __init__(self, filepath):
        self.filepath = filepath
        self.cap = cv.VideoCapture(self.filepath)

        if not self.cap.isOpened():
            print(f"❌ Error: Failed to open video file: {self.filepath}")
            sys.exit(1)

        self.latest_frame = None
        self._new_frame_available = False
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        print(f"✅ Video file opened: {self.filepath}")

    def _reader(self):
        """ Continuously read frames from the file. """
        while True:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            self.latest_frame = frame
            self._new_frame_available = True
            time.sleep(0.01)

    def frame_available(self):
        return self._new_frame_available

    def frame(self):
        self._new_frame_available = False
        return self.latest_frame

# --- Image Processing Functions ---
def isolate_line_color(image, lower_hsv, upper_hsv):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, np.array(lower_hsv), np.array(upper_hsv))
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    return cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

def extract_centerline(binary, min_dist=5):
    dist = cv.distanceTransform(binary, cv.DIST_L2, 5)
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv.dilate(dist, kernel)
    local_max = (dist == dilated) & (dist >= min_dist)
    thin = np.uint8(local_max * 255)
    thick_kernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    return cv.dilate(thin, thick_kernel, iterations=1)

def compute_line_angle(line):
    (x1, y1), (x2, y2) = line
    return -math.atan2(x2 - x1, y2 - y1) % math.pi

def cluster_lines_by_angle(lines, angle_thresh=0.3):
    if not lines:
        return []
    lines = sorted(lines, key=compute_line_angle)
    clusters = []
    curr = [lines[0]]
    ang = compute_line_angle(lines[0])
    for ln in lines[1:]:
        a = compute_line_angle(ln)
        if min(abs(a - ang), math.pi - abs(a - ang)) < angle_thresh:
            curr.append(ln)
            ang = np.mean([compute_line_angle(l) for l in curr])
        else:
            clusters.append(curr)
            curr = [ln]
            ang = a
    clusters.append(curr)
    return clusters

def merge_cluster(cluster):
    pts = np.array([p for ln in cluster for p in ln], dtype=np.float32)
    if len(pts) < 2:
        return None
    vx, vy, x0, y0 = cv.fitLine(pts, cv.DIST_L2, 0, 0.01, 0.01)
    vec = (vx[0], vy[0])
    projections = [(pt[0] - x0[0]) * vec[0] + (pt[1] - y0[0]) * vec[1] for pt in pts]
    mn, mx = min(projections), max(projections)
    p1 = (int(x0[0] + mn * vec[0]), int(y0[0] + mn * vec[1]))
    p2 = (int(x0[0] + mx * vec[0]), int(y0[0] + mx * vec[1]))
    return (p1, p2)

def compute_phi_e(p1, p2):
    phi_pipe = math.atan2(p2[0] - p1[0], p2[1] - p1[1])
    phi = -phi_pipe
    while phi < -math.pi/2:
        phi += math.pi
    while phi > math.pi/2:
        phi -= math.pi
    return phi

# --- Main Processing Loop ---
def main():
    parser = argparse.ArgumentParser(
        description="Process video file for docking-line detection"
    )
    parser.add_argument('video_file', help='Path to input video file')
    parser.add_argument('--pause', type=int, default=30,
        help='ms to wait between frames (0 = manual advance)')
    parser.add_argument('--lower-hsv', type=int, nargs=3,
        default=[20, 100, 100], help='Lower HSV for line color')
    parser.add_argument('--upper-hsv', type=int, nargs=3,
        default=[35, 255, 255], help='Upper HSV for line color')
    parser.add_argument('--win-size', type=float, default=0.5,
        help='Scale factor for display windows (0<factor<=1)')
    args = parser.parse_args()

    video = VideoFile(args.video_file)
    win_names = ["Original", "Mask", "Centerline", "Annotated"]
    for name in win_names:
        cv.namedWindow(name, cv.WINDOW_NORMAL)

    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set("formula2boat", "formula2boat")
    try:
        mqtt_client.connect("100.78.45.94", 1883)
    except:
        pass

    while True:
        if not video.frame_available():
            time.sleep(0.01)
            continue
        frame = video.frame()
        if frame is None:
            time.sleep(0.01)
            continue

        # Prepare display
        scale = args.win_size
        disp = cv.resize(frame, (0, 0), fx=scale, fy=scale)
        h_disp, w_disp = disp.shape[:2]
        center_disp_x = w_disp // 2

        # Process on full-res frame
        mask = isolate_line_color(frame, args.lower_hsv, args.upper_hsv)
        centerline = extract_centerline(mask, min_dist=3)

        # Hough detection
        linesP = cv.HoughLinesP(centerline, 1, np.pi/180, threshold=10,
                                 minLineLength=50, maxLineGap=20)

        # Default no detection
        y_e, phi_deg, detected = 0, 0.0, False
        pt1, pt2 = None, None
        bottom_pt = None

        if linesP is not None:
            hough = [((l[0][0], l[0][1]), (l[0][2], l[0][3])) for l in linesP]
            clusters = cluster_lines_by_angle(hough, angle_thresh=0.4)
            merged = [merge_cluster(c) for c in clusters]
            finals = [m for m in merged if m is not None]
            if finals:
                # choose the lowest line
                finals.sort(key=lambda ln: max(ln[0][1], ln[1][1]), reverse=True)
                pt1, pt2 = finals[0]
                phi = compute_phi_e(pt1, pt2)
                phi_deg = math.degrees(phi)
                bottom_pt = pt1 if pt1[1] > pt2[1] else pt2
                y_e = bottom_pt[0] - frame.shape[1] // 2
                detected = True

        # Annotate on scaled copy
        ann = disp.copy()
        cv.line(ann, (center_disp_x, 0), (center_disp_x, h_disp), (255, 0, 0), 1)
        if detected and pt1 and pt2:
            # scale coordinates for drawing
            p1 = (int(pt1[0] * scale), int(pt1[1] * scale))
            p2 = (int(pt2[0] * scale), int(pt2[1] * scale))
            bp = (int(bottom_pt[0] * scale), int(bottom_pt[1] * scale))
            cv.line(ann, p1, p2, (0, 255, 0), 2)
            cv.circle(ann, bp, 5, (0, 0, 255), -1)
        cv.putText(ann, f"Y: {y_e}px", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv.putText(ann, f"Phi: {phi_deg:.1f}deg", (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Display windows
        cv.imshow("Original", disp)
        cv.imshow("Mask", cv.resize(mask, (0, 0), fx=scale, fy=scale))
        cv.imshow("Centerline", cv.resize(centerline, (0, 0), fx=scale, fy=scale))
        cv.imshow("Annotated", ann)

        if cv.waitKey(args.pause) & 0xFF == ord('q'):
            break

    print("🔚 End of video. Press any key to exit.")
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
