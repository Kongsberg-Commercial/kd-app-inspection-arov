#!/usr/bin/env python3
import sys
import math
import time
import threading
import json
import argparse
import subprocess

import cv2 as cv
import numpy as np

# optional MQTT import
try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

class VideoStream:
    """UDP network stream: prefer FFmpeg → rawpipe, fallback to OpenCV."""
    def __init__(self, port, width=None, height=None, fifo_size=5_000_000):
        self.width = width
        self.height = height
        self.frame = None
        self.new = False
        self.alive = True

        url = f'udp://127.0.0.1:{port}?fifo_size={fifo_size}&overrun_nonfatal=1'

        # First attempt: FFmpeg raw‐pipe (requires ffmpeg in PATH)
        if width and height:
            cmd = [
                'ffmpeg',
                '-i', url,
                '-loglevel', 'quiet',
                '-f', 'rawvideo',
                '-pix_fmt', 'bgr24',
                '-'
            ]
            try:
                self.proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, bufsize=10**8
                )
                threading.Thread(target=self._reader_ffmpeg, daemon=True).start()
                return
            except FileNotFoundError:
                print("⚠️  Warning: 'ffmpeg' not found in PATH—falling back to OpenCV VideoCapture")

        # Fallback: OpenCV VideoCapture with FFmpeg backend
        self.cap = cv.VideoCapture(url, cv.CAP_FFMPEG)
        if not self.cap.isOpened():
            print(f"❌ Error: failed to open stream on port {port} via VideoCapture")
            sys.exit(1)
        threading.Thread(target=self._reader_cv, daemon=True).start()

    def _reader_ffmpeg(self):
        """Read raw bgr24 frames from ffmpeg stdout."""
        bytes_per = self.width * self.height * 3
        while self.alive:
            raw = self.proc.stdout.read(bytes_per)
            if not raw or len(raw) < bytes_per:
                break
            img = np.frombuffer(raw, np.uint8).reshape((self.height, self.width, 3))
            self.frame, self.new = img, True
        self.alive = False

    def _reader_cv(self):
        """Read frames via OpenCV VideoCapture."""
        while self.alive:
            ret, f = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            self.frame, self.new = f, True

    def frame_available(self):
        return self.new

    def get_frame(self):
        self.new = False
        return self.frame

    def terminate(self):
        self.alive = False
        if hasattr(self, 'proc'):
            self.proc.terminate()

def isolate_color(img, lo, hi):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, np.array(lo), np.array(hi))
    k = cv.getStructuringElement(cv.MORPH_RECT, (5,5))
    return cv.morphologyEx(cv.morphologyEx(mask, cv.MORPH_CLOSE, k),
                           cv.MORPH_OPEN, k)

def extract_centerline(bin_img, min_dist=5):
    dist = cv.distanceTransform(bin_img, cv.DIST_L2, 5)
    k = np.ones((3,3), np.uint8)
    dil = cv.dilate(dist, k)
    local_max = (dist == dil) & (dist >= min_dist)
    thin = np.uint8(local_max) * 255
    return cv.dilate(thin, cv.getStructuringElement(cv.MORPH_RECT, (7,7)))

def compute_phi(p1, p2):
    phi = -math.atan2(p2[0]-p1[0], p2[1]-p1[1])
    # normalize to [-90°, +90°]
    while phi < -math.pi/2: phi += math.pi
    while phi >  math.pi/2: phi -= math.pi
    return phi

def cluster_lines_by_angle(lines, thresh=0.3):
    if not lines: return []
    lines = sorted(lines, key=lambda ln: compute_phi(ln[0], ln[1]))
    clusters, curr = [], [lines[0]]
    ang = compute_phi(*lines[0])
    for ln in lines[1:]:
        a = compute_phi(*ln)
        if min(abs(a-ang), math.pi-abs(a-ang)) < thresh:
            curr.append(ln)
            ang = np.mean([compute_phi(*l) for l in curr])
        else:
            clusters.append(curr)
            curr, ang = [ln], a
    clusters.append(curr)
    return clusters

def merge_cluster(cluster):
    pts = np.array([p for ln in cluster for p in ln], dtype=np.float32)
    if len(pts) < 2:
        return None
    vx, vy, x0, y0 = cv.fitLine(pts, cv.DIST_L2, 0, 0.01, 0.01)
    vec = (vx[0], vy[0])
    projs = [(p[0]-x0[0])*vec[0] + (p[1]-y0[0])*vec[1] for p in pts]
    mn, mx = min(projs), max(projs)
    p1 = (int(x0[0] + mn*vec[0]), int(y0[0] + mn*vec[1]))
    p2 = (int(x0[0] + mx*vec[0]), int(y0[0] + mx*vec[1]))
    return p1, p2

def main():
    parser = argparse.ArgumentParser(
        description="Docking with vertical‐only line detection, FFmpeg UDP piping")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--stream-port', type=int,
                       help="UDP port for video (uses FFmpeg raw pipe)")
    group.add_argument('--video-file', help="Path to video file")
    parser.add_argument('--width',  type=int,
                        help="[stream only] frame width in pixels")
    parser.add_argument('--height', type=int,
                        help="[stream only] frame height in pixels")
    parser.add_argument('--pause',   type=int,   default=30,
                        help="ms between frames")
    parser.add_argument('--win-size',type=float, default=0.5,
                        help="Display scale factor")
    parser.add_argument('--no-mqtt', action='store_true',
                        help="Disable MQTT")
    args = parser.parse_args()

    is_file = bool(args.video_file)
    if args.stream_port:
        if args.width is None or args.height is None:
            print("❌ When using --stream-port you must also specify --width and --height")
            sys.exit(1)
        video = VideoStream(args.stream_port, args.width, args.height)
    else:
        cap = cv.VideoCapture(args.video_file)
        if not cap.isOpened():
            print(f"❌ Error: failed to open file {args.video_file}")
            sys.exit(1)
        fps = cap.get(cv.CAP_PROP_FPS) or 30
        skip = int(fps * 5)
        total = cap.get(cv.CAP_PROP_FRAME_COUNT)

    # create windows
    for w in ["Original","Line Mask","Square Mask",
              "Centerline","Annotated","Controls"]:
        cv.namedWindow(w, cv.WINDOW_NORMAL)

    def tb(name, default, maximum):
        cv.createTrackbar(name, "Controls", default, maximum, lambda x: None)

    # HSV & shape filters
    tb('L1_H_min', 26, 179); tb('L1_H_max', 38, 179)
    tb('L1_S_min', 35, 255); tb('L1_S_max',115, 255)
    tb('L1_V_min',165, 255); tb('L1_V_max',255, 255)
    tb('L2_H_min', 12, 179); tb('L2_H_max', 38, 179)
    tb('L2_S_min',  5, 255); tb('L2_S_max', 50, 255)
    tb('L2_V_min',100, 255); tb('L2_V_max',255, 255)
    tb('Area_min',6200,10000)
    tb('Aspect*10',31,50)

    # smoothing, priority, vertical tolerance
    tb('Smooth%',100,100)
    tb('Priority', 51,100)
    tb('AngTol',  12,90)  # degrees from vertical (0°) you’ll accept

    paused = False
    last = None
    prev_y = prev_phi = None

    # MQTT setup
    if mqtt and not args.no_mqtt:
        import socket
        socket.setdefaulttimeout(3)
        client = mqtt.Client()
        client.username_pw_set("formula2boat","formula2boat")
        client.loop_start()
        try:
            client.connect_async("100.86.23.108", 1883, keepalive=60)
        except:
            args.no_mqtt = True

    try:
        while True:
            key = cv.waitKey(1 if paused else args.pause) & 0xFF
            if key == ord('q'):
                break
            if key == ord('p'):
                paused = not paused
                print("Paused" if paused else "Running")

            if is_file:
                if key == ord('r'):
                    pos = int(cap.get(cv.CAP_PROP_POS_FRAMES))
                    cap.set(cv.CAP_PROP_POS_FRAMES, max(pos-skip,0))
                    continue
                if key == ord('f'):
                    pos = int(cap.get(cv.CAP_PROP_POS_FRAMES))
                    cap.set(cv.CAP_PROP_POS_FRAMES, min(pos+skip,total-1))
                    continue

            # grab frame
            if not paused:
                if is_file:
                    ret, frame = cap.read()
                    if not ret:
                        print("🔁 End of file.")
                        continue
                else:
                    if not video.frame_available():
                        continue
                    frame = video.get_frame()
                last = frame.copy()

            if last is None:
                continue
            frame = last.copy()

            # read controls
            l1 = [cv.getTrackbarPos(n,"Controls") for n in ('L1_H_min','L1_S_min','L1_V_min')]
            u1 = [cv.getTrackbarPos(n,"Controls") for n in ('L1_H_max','L1_S_max','L1_V_max')]
            l2 = [cv.getTrackbarPos(n,"Controls") for n in ('L2_H_min','L2_S_min','L2_V_min')]
            u2 = [cv.getTrackbarPos(n,"Controls") for n in ('L2_H_max','L2_S_max','L2_V_max')]
            area_min = cv.getTrackbarPos('Area_min',"Controls")
            aspect_max= cv.getTrackbarPos('Aspect*10',"Controls")/10.0
            smooth_p = cv.getTrackbarPos('Smooth%',"Controls")/100.0
            priority = cv.getTrackbarPos('Priority',"Controls")
            ang_tol  = cv.getTrackbarPos('AngTol',"Controls")

            # build masks & show them
            mask_line   = isolate_color(frame, l1, u1)
            mask_square = isolate_color(frame, l2, u2)
            cv.imshow("Original",    cv.resize(frame,       (0,0), fx=args.win_size, fy=args.win_size))
            cv.imshow("Line Mask",   cv.resize(mask_line,   (0,0), fx=args.win_size, fy=args.win_size))
            cv.imshow("Square Mask", cv.resize(mask_square, (0,0), fx=args.win_size, fy=args.win_size))

            # square detection
            cnts, _ = cv.findContours(mask_square, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            sq_det = False; sx = sy = 0; rect = None
            if cnts:
                c = max(cnts, key=cv.contourArea)
                if cv.contourArea(c) >= area_min:
                    rect = cv.minAreaRect(c)
                    (cx, cy), (w, h), _ = rect
                    if max(w, h)/(min(w, h)+1e-6) <= aspect_max:
                        sq_det, sx, sy = True, int(cx), int(cy)

            # line detection + vertical filter
            ctr = extract_centerline(mask_line, min_dist=3)
            lines = cv.HoughLinesP(ctr, 1, np.pi/180, 10, minLineLength=50, maxLineGap=20)
            ln_det = False; y_line = 0; phi_line = 0.0; p1 = p2 = None
            if lines is not None:
                hls = [((l[0][0],l[0][1]), (l[0][2],l[0][3])) for l in lines]
                clusters = cluster_lines_by_angle(hls, 0.4)
                merged = [merge_cluster(c) for c in clusters if merge_cluster(c)]
                if merged:
                    merged.sort(key=lambda ln: max(ln[0][1], ln[1][1]), reverse=True)
                    p1, p2 = merged[0]
                    phi_deg = math.degrees(compute_phi(p1, p2))
                    if abs(phi_deg) <= ang_tol:
                        phi_line = phi_deg
                        bot = p1 if p1[1]>p2[1] else p2
                        y_line = bot[0] - frame.shape[1]//2
                        ln_det = True

            # pick square vs line
            if priority < 50:
                use_sq = sq_det
            else:
                use_sq = sq_det and not ln_det

            if use_sq:
                cur_y, cur_phi, det = sx - frame.shape[1]//2, 0.0, True
            elif ln_det:
                cur_y, cur_phi, det = y_line, phi_line, True
            else:
                cur_y, cur_phi, det = 0, 0.0, False

            # exponential smoothing
            if prev_y is None:
                sm_y, sm_phi = cur_y, cur_phi
            else:
                sm_y   = prev_y*(1-smooth_p) + cur_y*smooth_p
                sm_phi = prev_phi*(1-smooth_p) + cur_phi*smooth_p
            prev_y, prev_phi = sm_y, sm_phi

            # MQTT publish
            msg = {
                "distance": int(sm_y),
                "angle":    round(sm_phi, 2),
                "detected": bool(det),
                "timestamp": time.time()
            }
            if mqtt and not args.no_mqtt:
                client.publish("/docking/line", json.dumps(msg))

            # annotate & show
            disp = cv.resize(frame, (0,0), fx=args.win_size, fy=args.win_size)
            ann = disp.copy()
            h_d, w_d = disp.shape[:2]
            cv.line(ann, (w_d//2,0), (w_d//2,h_d), (255,0,0), 1)

            if use_sq and rect is not None:
                box = cv.boxPoints(rect).astype(int)
                box = (box * args.win_size).astype(int)
                cv.drawContours(ann, [box], 0, (0,255,0), 2)
                cv.circle(ann, (int(sx*args.win_size), int(sy*args.win_size)), 5, (0,0,255), -1)
            elif det and p1 is not None and p2 is not None:
                pt1 = (int(p1[0]*args.win_size), int(p1[1]*args.win_size))
                pt2 = (int(p2[0]*args.win_size), int(p2[1]*args.win_size))
                cv.line(ann, pt1, pt2, (0,255,0), 2)
                bp = (
                    int((p1 if p1[1]>p2[1] else p2)[0]*args.win_size),
                    int((p1 if p1[1]>p2[1] else p2)[1]*args.win_size)
                )
                cv.circle(ann, bp, 5, (0,0,255), -1)

            cv.putText(ann, f"Y: {int(sm_y)}px", (10,30),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            if ln_det:
                cv.putText(ann, f"Phi: {sm_phi:.1f}deg", (10,60),
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            cv.imshow("Annotated",   ann)
            cv.imshow("Centerline",  cv.resize(ctr, (0,0), fx=args.win_size, fy=args.win_size))

    finally:
        if args.stream_port:
            video.terminate()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()