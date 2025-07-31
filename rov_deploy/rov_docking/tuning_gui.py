import cv2 as cv
import numpy as np
import time
def nothing(x): pass

cv.namedWindow('Trackbars', cv.WINDOW_NORMAL)
for name, val in [('H_min',20),('H_max',35),('S_min',150),
                  ('S_max',255),('V_min',150),('V_max',255)]:
    cv.createTrackbar(name, 'Trackbars', val, 255, nothing)

cap = cv.VideoCapture('2025-07-30_13.36.08.mkv')  # or 0 for webcam
paused = False

while True:
    if not paused:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

    # Read slider positions
    h_min = cv.getTrackbarPos('H_min','Trackbars')
    h_max = cv.getTrackbarPos('H_max','Trackbars')
    s_min = cv.getTrackbarPos('S_min','Trackbars')
    s_max = cv.getTrackbarPos('S_max','Trackbars')
    v_min = cv.getTrackbarPos('V_min','Trackbars')
    v_max = cv.getTrackbarPos('V_max','Trackbars')

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv.inRange(hsv, lower, upper)

    cv.imshow('Original', frame)
    cv.imshow('Mask', mask)

    key = cv.waitKey(30) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('p'):
        paused = not paused  # toggle pause

cap.release()
cv.destroyAllWindows()
