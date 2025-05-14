# This is the main image processing script. It fetches frames from the video feed using gstreamer and passes it to OpenCV2. We then extract the white-ish pixels corresponding to the pipe, find the "centerline" or "skeleton" of the pipe, and run probabilistic hough line transform. We find phi_e and y_e and send it to the servers.
# We also draw detections on the images, compress to jpg and send it to the topside computer on its own port (8889)
# To run this script, the environment at ~/python/testing/mavlink_testing/env has to be sourced, or the binary ~/python/testing/mavlink_testing/env/bin/python3 has to be used. For example:
# ~/python/testing/mavlink_testing/env/bin/python3 videoprocessor.py
# Also make sure to adjust the socket connections based on what is actually listening!
import sys
import math
import cv2 as cv
import numpy as np
import time
import gi
import socket
import struct
import paho.mqtt.client as mqtt
import json

gi.require_version('Gst', '1.0')
from gi.repository import Gst

# boilerplate from https://www.ardusub.com/developers/opencv.html
class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, port=5601):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (
                caps_structure.get_value('height'),
                caps_structure.get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """ Get frame to update _new_frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK


# Isolate white-ish regions corresponding to the pipe.
def isolate_pipe(image):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([255, 255, 255])
    mask = cv.inRange(hsv, lower_white, upper_white)

    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    mask_closed = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask_clean = cv.morphologyEx(mask_closed, cv.MORPH_OPEN, kernel)

    isolated = cv.bitwise_and(image, image, mask=mask_clean)
    return isolated, mask_clean

def extract_centerline(binary, min_dist=5):
    dist = cv.distanceTransform(binary, cv.DIST_L2, 5)
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv.dilate(dist, kernel)
    local_max = (dist == dilated)
    local_max = np.logical_and(local_max, dist >= min_dist)
    thin_centerline = np.uint8(local_max * 255)
    thick_kernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    thick_centerline = cv.dilate(thin_centerline, thick_kernel, iterations=1)
    return thick_centerline

def compute_line_angle(line):
    (x1, y1), (x2, y2) = line
    phi = -math.atan2(x2 - x1,y2 - y1) % math.pi
    return phi

def cluster_lines_by_angle(lines, angle_thresh=0.3):
    if not lines:
        return []
    lines = sorted(lines, key=lambda l: compute_line_angle(l))
    clusters = []
    current_cluster = [lines[0]]
    current_angle = compute_line_angle(lines[0])
    for line in lines[1:]:
        angle = compute_line_angle(line)
        diff = min(abs(angle - current_angle), math.pi - abs(angle - current_angle))
        if diff < angle_thresh:
            current_cluster.append(line)
            current_angle = (current_angle * (len(current_cluster)-1) + angle) / len(current_cluster)
        else:
            clusters.append(current_cluster)
            current_cluster = [line]
            current_angle = angle
    if current_cluster:
        clusters.append(current_cluster)
    return clusters

def merge_cluster(cluster):
    if not cluster:
        return None
    points = []
    for (p1, p2) in cluster:
        points.append(p1)
        points.append(p2)
    points = np.array(points, dtype=np.float32)
    sin_sum = 0.0
    cos_sum = 0.0
    for (p1, p2) in cluster:
        phi = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        sin_sum += math.sin(phi)
        cos_sum += math.cos(phi)
    avg_phi = math.atan2(sin_sum, cos_sum)
    direction = np.array([math.cos(avg_phi), math.sin(avg_phi)])
    projections = [np.dot(pt, direction) for pt in points]
    min_proj = min(projections)
    max_proj = max(projections)
    center = np.mean(points, axis=0)
    pt_min = (int(center[0] + (min_proj - np.dot(center, direction)) * direction[0]),
              int(center[1] + (min_proj - np.dot(center, direction)) * direction[1]))
    pt_max = (int(center[0] + (max_proj - np.dot(center, direction)) * direction[0]),
              int(center[1] + (max_proj - np.dot(center, direction)) * direction[1]))
    return (pt_min, pt_max)

def compute_phi_e(p1, p2):
    phi_pipe = math.atan2(p2[0] - p1[0], p2[1] - p1[1])
    phi_e = -phi_pipe
    while phi_e < -math.pi/2:
        phi_e += math.pi
    while phi_e > math.pi/2:
        phi_e -= math.pi
    return phi_e

def connect_socket(ip, port):
    """Attempt to connect to the given IP and port. Retry until successful."""
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((ip, port))
            print(f"Connected to {ip}:{port}")
            return sock
        except Exception as e:
            print(f"Connection to {ip}:{port} failed with error: {e}. Retrying in 1 second.")
            time.sleep(1)

def main(argv):
    video = Video(port=5601)
    sock_local, sock_remote, sock_video = None, None, None  
    # Establish initial socket connections.
    REMOTEIP = "192.168.2.1"
    VIDEOPORT = 8889
    #sock_video = connect_socket(REMOTEIP, VIDEOPORT)

    # Connect to MQTT
    mqtt_client = mqtt.Client()
    mqtt_client.connect("localhost", 1883)  # Or replace with your broker IP

    print("Established connections with server(s).")

    while True:
        if not video.frame_available():
            continue

        try:
            image = video.frame()
            start_time = time.time()

            # Process the image.
            isolated, _ = isolate_pipe(image)
            gray = cv.cvtColor(isolated, cv.COLOR_BGR2GRAY)
            _, binary = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
            centerline = extract_centerline(binary, min_dist=3)
            linesP = cv.HoughLinesP(centerline, 1, np.pi/180, threshold=10, minLineLength=30, maxLineGap=10)
            hough_lines = []
            if linesP is not None:
                for i in range(len(linesP)):
                    l = linesP[i][0]
                    hough_lines.append(((l[0], l[1]), (l[2], l[3])))
            else:
                print("No Hough segments detected on the centerline.")
                continue

            clusters = cluster_lines_by_angle(hough_lines, angle_thresh=0.3)
            final_lines = []
            for cluster in clusters:
                merged = merge_cluster(cluster)
                if merged is not None:
                    final_lines.append(merged)

            img_center_x = int(image.shape[1] / 2)
            if final_lines:
                pt1, pt2 = max(final_lines, key=lambda line: max(line[0][1], line[1][1]))
                phi_e = compute_phi_e(pt1, pt2)
                phi_e_deg = math.degrees(phi_e)
                bottom_pt = pt1 if pt1[1] > pt2[1] else pt2
                y_e = bottom_pt[0] - img_center_x
                data_str = f"{y_e};{phi_e_deg}\n" 
            else:
                print("No final lines detected.")
                continue

            pipe_msg = {
                        "distance": int(y_e),               # lateral offset in pixels
                        "angle": round(phi_e_deg, 2),       # angle in degrees
                        "timestamp": time.time()
                        }

            mqtt_client.publish("/pipe", json.dumps(pipe_msg))


            if sock_video is not None:
                # Draw detections on the image.
                out_img = image.copy()
                cv.line(out_img, (img_center_x, 0), (img_center_x, image.shape[0]), (255, 0, 0), 2, cv.LINE_AA)
                for idx, (pt1, pt2) in enumerate(final_lines):
                    cv.line(out_img, pt1, pt2, (0, 255, 0), 3, cv.LINE_AA)
                    bottom_pt = pt1 if pt1[1] > pt2[1] else pt2
                    cv.circle(out_img, bottom_pt, 5, (255, 0, 0), -1)
                    center_dot = (img_center_x, bottom_pt[1])
                    cv.circle(out_img, center_dot, 5, (255, 0, 0), -1)
                    cv.line(out_img, bottom_pt, center_dot, (255, 0, 0), 2, cv.LINE_AA)

                # Encode the processed image and send it via the video socket.
                retval, buffer = cv.imencode('.jpg', out_img)
                data = buffer.tobytes()
                header = struct.pack('>I', len(data))  # Send the length of the image data
                try:
                    sock_video.sendall(header + data)
                except Exception as e:
                    print(f"Error sending image on sock_video: {e}")
                    sock_video.close()
                    sock_video = connect_socket(REMOTEIP, VIDEOPORT)

            print(f"Processing time: {time.time() - start_time} seconds")
        except Exception as e:
            print(f"Error processing frame: {e}")

if __name__ == "__main__":
    main(sys.argv[1:])

