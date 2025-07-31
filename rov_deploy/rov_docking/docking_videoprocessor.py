import sys
import math
import cv2 as cv
import numpy as np
import time
import threading
import paho.mqtt.client as mqtt
import json
import argparse

# --- Video classes ---
class VideoStream:
    """
    Network stream video using cv2.VideoCapture over UDP.
    """
    def __init__(self, port, retry_delay=1.0):
        self.video_url = f'udp://0.0.0.0:{port}?overrun_nonfatal=1&fifo_size=50000000'
        self.cap = cv.VideoCapture(self.video_url, cv.CAP_FFMPEG)
        if not self.cap.isOpened():
            print(f"❌ Error: failed to open stream on port {port}")
            sys.exit(1)
        self.latest_frame = None
        self._new = False
        self.retry_delay = retry_delay
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        print(f"✅ UDP stream opened on port {port}")

    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Stream disconnected, retrying...")
                self.cap.release()
                time.sleep(self.retry_delay)
                self.cap = cv.VideoCapture(self.video_url, cv.CAP_FFMPEG)
                continue
            self.latest_frame = frame
            self._new = True
            time.sleep(0.01)

    def frame_available(self):
        return self._new

    def frame(self):
        self._new = False
        return self.latest_frame

class VideoFile:
    """
    Local file input using cv2.VideoCapture.
    """
    def __init__(self, filepath):
        self.cap = cv.VideoCapture(filepath)
        if not self.cap.isOpened():
            print(f"❌ Error: failed to open file {filepath}")
            sys.exit(1)
        self.latest_frame = None
        self._new = False
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        print(f"✅ Video file opened: {filepath}")

    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            self.latest_frame = frame
            self._new = True
            time.sleep(0.01)

    def frame_available(self):
        return self._new

    def frame(self):
        self._new = False
        return self.latest_frame

# --- Image processing ---
def isolate_line_color(img, lower_hsv, upper_hsv):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, np.array(lower_hsv), np.array(upper_hsv))
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    return cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

def extract_centerline(bin_img, min_dist=5):
    dist = cv.distanceTransform(bin_img, cv.DIST_L2, 5)
    kernel = np.ones((3,3), np.uint8)
    dilated = cv.dilate(dist, kernel)
    local_max = (dist == dilated) & (dist >= min_dist)
    thin = np.uint8(local_max) * 255
    return cv.dilate(thin, cv.getStructuringElement(cv.MORPH_RECT, (7,7)))

def compute_line_angle(line):
    (x1, y1), (x2, y2) = line
    return -math.atan2(x2-x1, y2-y1) % math.pi

def cluster_lines_by_angle(lines, angle_thresh=0.3):
    if not lines: return []
    lines = sorted(lines, key=compute_line_angle)
    clusters = []
    curr = [lines[0]]
    ang = compute_line_angle(lines[0])
    for ln in lines[1:]:
        a = compute_line_angle(ln)
        diff = min(abs(a-ang), math.pi-abs(a-ang))
        if diff < angle_thresh:
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
    if len(pts) < 2: return None
    vx, vy, x0, y0 = cv.fitLine(pts, cv.DIST_L2, 0, 0.01, 0.01)
    vec = (vx[0], vy[0])
    projs = [(p[0]-x0[0])*vec[0] + (p[1]-y0[0])*vec[1] for p in pts]
    mn, mx = min(projs), max(projs)
    p1 = (int(x0[0]+mn*vec[0]), int(y0[0]+mn*vec[1]))
    p2 = (int(x0[0]+mx*vec[0]), int(y0[0]+mx*vec[1]))
    return p1, p2

def compute_phi_e(p1, p2):
    phi = -math.atan2(p2[0]-p1[0], p2[1]-p1[1])
    while phi < -math.pi/2: phi += math.pi
    while phi > math.pi/2: phi -= math.pi
    return phi

# --- Main ---
def main():
    parser = argparse.ArgumentParser(description="Docking video processor")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--stream-port', type=int,
                       help='UDP port to read H.264 stream (e.g. 5600)')
    group.add_argument('--video-file', help='Path to input video file')
    parser.add_argument('--lower-hsv', type=int, nargs=3, default=[20,100,100],
                        help='Lower HSV threshold')
    parser.add_argument('--upper-hsv', type=int, nargs=3, default=[35,255,255],
                        help='Upper HSV threshold')
    parser.add_argument('--pause', type=int, default=30,
                        help='Wait ms between frames (0=manual)')
    parser.add_argument('--win-size', type=float, default=0.5,
                        help='Display scale factor (0< <=1)')
    parser.add_argument('--no-mqtt', action='store_true',
                        help="Disable MQTT publishing")
    args = parser.parse_args()

    if args.stream_port:
        video = VideoStream(port=args.stream_port)
    else:
        video = VideoFile(filepath=args.video_file)

    # Create resizable windows
    names = ["Original","Mask","Centerline","Annotated"]
    for n in names:
        cv.namedWindow(n, cv.WINDOW_NORMAL)

    # Setup MQTT if enabled
    if not args.no_mqtt:
            import socket
            socket.setdefaulttimeout(3)             # optional extra safety
            mqtt_client = mqtt.Client()
            mqtt_client.username_pw_set("formula2boat","formula2boat")
            mqtt_client.loop_start()                # <— start background thread
            try:
                mqtt_client.connect_async(
                    "100.78.45.94",
                    1883,
                    keepalive=60
                )
            except Exception as e:
                print(f"❌ MQTT async‑connect failed: {e}")
                args.no_mqtt = True

    while True:
        if not video.frame_available():
            time.sleep(0.01); continue
        frame = video.frame()
        if frame is None: continue

        # full-res processing
        mask = isolate_line_color(frame, args.lower_hsv, args.upper_hsv)
        centerline = extract_centerline(mask, min_dist=3)
        linesP = cv.HoughLinesP(centerline,1,np.pi/180,threshold=10,
                                 minLineLength=50,maxLineGap=20)

        # default outputs
        y_e, phi_deg = 0, 0.0
        detected, pt1, pt2 = False, None, None

        if linesP is not None:
            hough = [((l[0][0],l[0][1]),(l[0][2],l[0][3])) for l in linesP]
            clusters = cluster_lines_by_angle(hough, angle_thresh=0.4)
            merged = [merge_cluster(c) for c in clusters]
            finals = [m for m in merged if m]
            if finals:
                finals.sort(key=lambda ln: max(ln[0][1],ln[1][1]), reverse=True)
                pt1, pt2 = finals[0]
                phi = compute_phi_e(pt1, pt2)
                phi_deg = math.degrees(phi)
                bottom = pt1 if pt1[1]>pt2[1] else pt2
                y_e = bottom[0] - frame.shape[1]//2
                detected = True

        line_msg = {
            "distance": int(y_e),
            "angle": round(phi_deg, 2),
            "detected": detected,
            "timestamp": time.time()
        }
        if not args.no_mqtt:
            try:
                mqtt_client.publish("/docking/line", json.dumps(line_msg))
            except Exception as e:
                print(f"❌ MQTT publish failed: {e}")
        # Annotate
        disp = cv.resize(frame, (0,0), fx=args.win_size, fy=args.win_size)
        h_disp, w_disp = disp.shape[:2]
        ann = disp.copy()
        cx = w_disp//2
        cv.line(ann,(cx,0),(cx,h_disp),(255,0,0),1)
        if detected and pt1 and pt2:
            p1 = (int(pt1[0]*args.win_size), int(pt1[1]*args.win_size))
            p2 = (int(pt2[0]*args.win_size), int(pt2[1]*args.win_size))
            bp = (int((pt1 if pt1[1]>pt2[1] else pt2)[0]*args.win_size),
                  int((pt1 if pt1[1]>pt2[1] else pt2)[1]*args.win_size))
            cv.line(ann,p1,p2,(0,255,0),2)
            cv.circle(ann,bp,5,(0,0,255),-1)
        cv.putText(ann,f"Y: {y_e}px",(10,30),cv.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
        cv.putText(ann,f"Phi: {phi_deg:.1f}deg",(10,60),cv.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)

        # Show all
        cv.imshow("Original",cv.resize(frame,(0,0),fx=args.win_size,fy=args.win_size))
        cv.imshow("Mask",cv.resize(mask,(0,0),fx=args.win_size,fy=args.win_size))
        cv.imshow("Centerline",cv.resize(centerline,(0,0),fx=args.win_size,fy=args.win_size))
        cv.imshow("Annotated",ann)

        key = cv.waitKey(args.pause) & 0xFF
        if key == ord('q'):
            break

    cv.destroyAllWindows()

if __name__ == "__main__":
    main()