import cv2
import numpy as np
import time


def stop():
    print('stop')
    time.sleep(0.5)  # simulate stop action delay

def forward():
    print('forward')
    time.sleep(0.5)  # simulate forward action delay

def left():
    print('left')
    time.sleep(0.5)  # simulate left action delay

def right():
    print('right')
    time.sleep(0.5)  # simulate right action delay

def detect_red_object(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # range
    lower_red = np.array([160, 100, 100])
    upper_red = np.array([179, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # detect red objects

    if contours:
        largest = max(contours, key=cv2.contourArea) #select largest red object
        area = cv2.contourArea(largest)
        # center co-ordinates 
        if area > 500:
            x, y, w, h = cv2.boundingRect(largest)
            cx = x + w // 2
            cy = y + h // 2
            return cx, cy, x, y, w, h
    return None, None, None, None, None, None

def track_object(cx, frame_center, tolerance=50):
    if cx is None:
        stop() # no red object
    elif abs(cx - frame_center) < tolerance: # object centered
        forward()
    elif cx < frame_center - tolerance: # object on left
        left()
    else:
        right()

# read cam
cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cx, cy, x, y, w, h = detect_red_object(frame)
        frame_center = frame.shape[1] // 2

        if cx is not None:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

        track_object(cx, frame_center)

        cv2.imshow("Red Object Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    stop()
    cap.release()
    cv2.destroyAllWindows()
