import sys
import math
import cv2 as cv
import numpy as np
import time

def isolate_pipe(image):
    #cv.imshow("HSV1", image)
    # Convert to HSV and define a white-ish range.
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    #cv.imshow("HSV2", hsv)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([255, 255, 255])
    mask = cv.inRange(hsv, lower_white, upper_white)
    #cv.imshow("HSV conversion", mask)
    # Morphological operations: close gaps then remove noise.
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
    mask_closed = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask_clean = cv.morphologyEx(mask_closed, cv.MORPH_OPEN, kernel)

    # Apply the mask to the original image.
    isolated = cv.bitwise_and(image, image, mask=mask_clean)
    return isolated, mask_clean

def extract_centerline(binary, min_dist=5):
    # Compute distance transform.
    dist = cv.distanceTransform(binary, cv.DIST_L2, 5)
    # Dilate the distance transform.
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv.dilate(dist, kernel)
    # Find local maxima: pixels equal to the dilated value.
    local_max = (dist == dilated)
    # Keep only pixels above a minimum distance threshold.
    local_max = np.logical_and(local_max, dist >= min_dist)
    thin_centerline = np.uint8(local_max * 255)
    # Dilate the thin centerline to get a thicker centerline.
    thick_kernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    thick_centerline = cv.dilate(thin_centerline, thick_kernel, iterations=1)
    return thick_centerline

def compute_line_angle(line):
    (x1, y1), (x2, y2) = line
    phi = math.atan2(y2 - y1, x2 - x1) % math.pi
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
    # Collect all endpoints.
    points = []
    for (p1, p2) in cluster:
        points.append(p1)
        points.append(p2)
    points = np.array(points, dtype=np.float32)
    # Compute the average angle of the cluster.
    sin_sum = 0.0
    cos_sum = 0.0
    for (p1, p2) in cluster:
        phi = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        sin_sum += math.sin(phi)
        cos_sum += math.cos(phi)
    avg_phi = math.atan2(sin_sum, cos_sum)
    direction = np.array([math.cos(avg_phi), math.sin(avg_phi)])
    # Project each point on the direction.
    projections = [np.dot(pt, direction) for pt in points]
    min_proj = min(projections)
    max_proj = max(projections)
    center = np.mean(points, axis=0)
    # Compute extreme endpoints relative to the center.
    pt_min = (int(center[0] + (min_proj - np.dot(center, direction)) * direction[0]),
              int(center[1] + (min_proj - np.dot(center, direction)) * direction[1]))
    pt_max = (int(center[0] + (max_proj - np.dot(center, direction)) * direction[0]),
              int(center[1] + (max_proj - np.dot(center, direction)) * direction[1]))
    return (pt_min, pt_max)

def compute_phi_e(p1, p2):
    phi_pipe = math.atan2(p2[0] - p1[0], p2[1] - p1[1])
    phi_e =- phi_pipe
    while phi_e < -math.pi/2:
        phi_e += math.pi
    while phi_e > math.pi/2:
        phi_e -= math.pi
    return phi_e

def main(argv):
    default_file = 'test.png'
    filename = argv[0] if len(argv) > 0 else default_file

    start_time = time.time()

    # Load the original image.
    image = cv.imread(filename)
    if image is None:
        print("Error opening image!")
        print(f"Usage: {sys.argv[0]} [image_name -- default {default_file}]")
        return -1

    # --- Step 1: Isolate the white pipe ---
    isolated, mask_clean = isolate_pipe(image)
    # Optionally, save the isolated image.
    #cv.imwrite("isolated.png", isolated)

    # Display the original image, the mask, and the isolated result.
    cv.imshow("Original Image", image)
    cv.imshow("White-ish Mask", mask_clean)
    #cv.imshow("Isolated Pipe", isolated)

    # --- Step 2: Process the isolated pipe for centerline and line detection ---
    # Convert the isolated image to grayscale and threshold to get a binary image.
    gray = cv.cvtColor(isolated, cv.COLOR_BGR2GRAY)
    _, binary = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)

    # Extract a thick centerline from the binary image.
    centerline = extract_centerline(binary, min_dist=3)

    # Run the Probabilistic Hough Transform on the centerline.
    linesP = cv.HoughLinesP(centerline, 1, np.pi/180, threshold=10, minLineLength=30, maxLineGap=10)
    hough_lines = []
    if linesP is not None:
        for i in range(len(linesP)):
            l = linesP[i][0]
            hough_lines.append(((l[0], l[1]), (l[2], l[3])))
    else:
        print("No Hough segments detected on the centerline.")

    # Cluster the segments by angle.
    clusters = cluster_lines_by_angle(hough_lines, angle_thresh=0.3)
    final_lines = []
    for cluster in clusters:
        merged = merge_cluster(cluster)
        if merged is not None:
            final_lines.append(merged)

    # Create an output image to draw results.
    out_img = image.copy()
    img_center_x = int(image.shape[1] / 2)
    cv.line(out_img, (img_center_x, 0), (img_center_x, image.shape[0]), (255, 0, 0), 2, cv.LINE_AA)

    # For each final line, draw it and compute parameters.
    for idx, (pt1, pt2) in enumerate(final_lines):
        print(f"Final line {idx}: (x1={pt1[0]}, y1={pt1[1]}), (x2={pt2[0]}, y2={pt2[1]})")
        cv.line(out_img, pt1, pt2, (0, 255, 0), 3, cv.LINE_AA)
        phi_e = compute_phi_e(pt1, pt2)
        phi_e_deg = math.degrees(phi_e)
        # Choose the bottom-most point (largest y value).
        bottom_pt = pt1 if pt1[1] > pt2[1] else pt2
        y_e = bottom_pt[0] - img_center_x
        print(f"  phi_e (radians): {phi_e:.3f} ({phi_e_deg:.1f}°)")
        print(f"  y_e (pixels): {y_e:.1f}")
        # Draw markers for the computed parameters.
        cv.circle(out_img, bottom_pt, 5, (255, 0, 0), -1)
        center_dot = (img_center_x, bottom_pt[1])
        cv.circle(out_img, center_dot, 5, (255, 0, 0), -1)
        cv.line(out_img, bottom_pt, center_dot, (255, 0, 0), 2, cv.LINE_AA)

    cv.line(out_img, (img_center_x, 0), (img_center_x, image.shape[0]), (255, 0, 0), 2, cv.LINE_AA)
    print(f"Processing time: {time.time() - start_time}")
    #print(max(final_lines, key=lambda line: max(line[0][1], line[1][1])) if final_lines else None)

    # Display the centerline mask and final results.
    cv.imshow("Centerline Mask", centerline)
    cv.imshow("Final Center Lines", out_img)

    # Loop until 'q' is pressed.
    while True:
        key = cv.waitKey(100) & 0xFF
        if key == ord('q'):
            break
    cv.destroyAllWindows()
    return 0

if __name__ == "__main__":
    main(sys.argv[1:])
