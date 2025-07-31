# docking_image_processor.py
# A test version of the docking video processor adapted to process still images from a directory or glob pattern.
# Now supports configurable HSV thresholds (e.g. yellow or white) to handle different line colors.

import sys
import os
import math
import cv2 as cv
import numpy as np
import time
import paho.mqtt.client as mqtt
import json
import argparse
import glob

# --- Image Iterator for file inputs ---
class ImageLoader:
    """
    Loads images matching a glob pattern or in a directory.
    Usage: loader = ImageLoader(pattern) or ImageLoader(dir_path)
    Then call loader.next_image() to get (path, image) tuples until None.
    """
    def __init__(self, source):
        if os.path.isdir(source):
            self.paths = sorted(glob.glob(os.path.join(source, '*')))
        else:
            self.paths = sorted(glob.glob(source))
        if not self.paths:
            print(f"❌ No images found for pattern or directory: {source}")
            sys.exit(1)
        self.index = 0
        print(f"✅ Found {len(self.paths)} images to process.")

    def next_image(self):
        if self.index >= len(self.paths):
            return None, None
        path = self.paths[self.index]
        img = cv.imread(path)
        self.index += 1
        if img is None:
            print(f"⚠️ Warning: Unable to read image: {path}")
            return self.next_image()
        return path, img

# --- Image Processing Functions ---
def isolate_line_color(image, lower_hsv, upper_hsv):
    """Isolate pixels within the given HSV range."""
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, np.array(lower_hsv), np.array(upper_hsv))
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    return cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

def extract_centerline(binary, min_dist=5):
    dist = cv.distanceTransform(binary, cv.DIST_L2, 5)
    kern = np.ones((3, 3), np.uint8)
    dil = cv.dilate(dist, kern)
    local_max = (dist == dil) & (dist >= min_dist)
    thin = np.uint8(local_max * 255)
    thick_k = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    return cv.dilate(thin, thick_k, iterations=1)

def compute_line_angle(line):
    (x1, y1), (x2, y2) = line
    return -math.atan2(x2 - x1, y2 - y1) % math.pi

def cluster_lines_by_angle(lines, angle_thresh=0.3):
    if not lines: return []
    lines = sorted(lines, key=compute_line_angle)
    clusters = []
    curr = [lines[0]]
    ang = compute_line_angle(lines[0])
    for ln in lines[1:]:
        a = compute_line_angle(ln)
        if min(abs(a-ang), math.pi-abs(a-ang)) < angle_thresh:
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
    vx, vy, x0, y0 = cv.fitLine(pts, cv.DIST_L2,0,0.01,0.01)
    vec = (vx[0], vy[0])
    proj = [(p[0]-x0[0])*vec[0] + (p[1]-y0[0])*vec[1] for p in pts]
    mn, mx = min(proj), max(proj)
    p1 = (int(x0[0]+mn*vec[0]), int(y0[0]+mn*vec[1]))
    p2 = (int(x0[0]+mx*vec[0]), int(y0[0]+mx*vec[1]))
    return (p1, p2)

def compute_phi_e(p1, p2):
    phi_pipe = math.atan2(p2[0]-p1[0], p2[1]-p1[1])
    phi = -phi_pipe
    while phi < -math.pi/2: phi += math.pi
    while phi >  math.pi/2: phi -= math.pi
    return phi

# --- Main ---
def main():
    parser = argparse.ArgumentParser(
        description="Process still images for docking-line detection"
    )
    parser.add_argument('source',
        help='Directory of images or glob pattern, e.g. "images/*.jpg"')
    parser.add_argument('--pause', type=int, default=500,
        help='ms to wait between images (0=manual advance)')
    parser.add_argument('--lower-hsv', type=int, nargs=3,
        default=[0,0,200],
        help='Lower HSV for line color (e.g. white [0,0,200])')
    parser.add_argument('--upper-hsv', type=int, nargs=3,
        default=[180,50,255],
        help='Upper HSV for line color (e.g. white [180,50,255])')
    args = parser.parse_args()

    loader = ImageLoader(args.source)
    # MQTT Config (optional)
    mqtt_client = mqtt.Client()
    mqtt_client.username_pw_set("formula2boat","formula2boat")
    try:
        mqtt_client.connect("100.78.45.94",1883)
    except:
        pass

    while True:
        path, image = loader.next_image()
        if image is None: break

        h,w = image.shape[:2]
        center_x = w//2

        # Mask using configurable HSV
        mask = isolate_line_color(image, args.lower_hsv, args.upper_hsv)
        cent = extract_centerline(mask, min_dist=3)
        lines = cv.HoughLinesP(cent,1,np.pi/180,10,50,20)

        y_e=0; phi_deg=0; det=False; pt1=pt2=bot=None
        if lines is not None:
            hl = [((l[0][0],l[0][1]),(l[0][2],l[0][3])) for l in lines]
            cls = cluster_lines_by_angle(hl,0.4)
            fl = [merge_cluster(c) for c in cls if c]
            if fl:
                fl.sort(key=lambda ln: max(ln[0][1],ln[1][1]), reverse=True)
                pt1,pt2 = fl[0]
                phi = compute_phi_e(pt1,pt2); phi_deg=math.degrees(phi)
                bot = pt1 if pt1[1]>pt2[1] else pt2
                y_e = bot[0]-center_x; det=True

        # Publish
        msg = {"distance":int(y_e),"angle":round(phi_deg,2),"detected":det,"file":os.path.basename(path)}
        try: mqtt_client.publish("/docking/line",json.dumps(msg))
        except: pass

        # Visualization
        ann = image.copy()
        cv.line(ann,(center_x,0),(center_x,h),(255,0,0),1)
        if det:
            cv.line(ann,pt1,pt2,(0,255,0),2)
            cv.circle(ann,bot,5,(0,0,255),-1)
        cv.putText(ann,f"Y: {y_e}px",(10,30),cv.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
        cv.putText(ann,f"Phi: {phi_deg:.1f}deg",(10,60),cv.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
        cv.putText(ann,os.path.basename(path),(10,h-10),cv.FONT_HERSHEY_SIMPLEX,0.6,(200,200,200),1)

        cv.imshow("Original",image)
        cv.imshow("Mask",mask)
        cv.imshow("Centerline",cent)
        cv.imshow("Annotated",ann)

        key = cv.waitKey(args.pause)&0xFF
        if key==ord('q'): break

    print("🔚 End. Press any key to exit.")
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__=="__main__": main()
