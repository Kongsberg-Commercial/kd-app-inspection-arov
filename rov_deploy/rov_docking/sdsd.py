import cv2 as cv

cap = cv.VideoCapture("udp://127.0.0.1:5601", cv.CAP_FFMPEG)
if not cap.isOpened():
    raise RuntimeError("Failed to open local bridge stream")

while True:
    ret, frame = cap.read()
    if not ret:
        print("No more frames, exiting")
        break
    cv.imshow("BlueROV2 via FFmpeg Bridge", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()