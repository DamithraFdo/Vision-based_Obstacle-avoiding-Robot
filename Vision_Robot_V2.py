import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import board
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

# Motor pins
motor1_in1 = 27
motor1_in2 = 22
motor2_in1 = 23
motor2_in2 = 24

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1_in1, GPIO.OUT)
GPIO.setup(motor1_in2, GPIO.OUT)
GPIO.setup(motor2_in1, GPIO.OUT)
GPIO.setup(motor2_in2, GPIO.OUT)

# Set motors to stop initially
GPIO.output(motor1_in1, GPIO.LOW)
GPIO.output(motor1_in2, GPIO.LOW)
GPIO.output(motor2_in1, GPIO.LOW)
GPIO.output(motor2_in2, GPIO.LOW)

# Initialize camera
cap = cv2.VideoCapture(0)

# Initialize I2C and OLED
i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_128x32(i2c)
oled.fill(0)
oled.show()

# Load YOLO model
model = YOLO('yolov5n.pt')

# Function to display status on OLED
def display_status(message):
    oled.fill(0)
    draw = ImageDraw.Draw(oled.image)
    font = ImageFont.load_default()
    draw.text((0, 0), message, font=font, fill=255)
    oled.show()

# Motor control functions
def move_forward():
    GPIO.output(motor1_in1, GPIO.HIGH)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in1, GPIO.HIGH)
    GPIO.output(motor2_in2, GPIO.LOW)
    display_status("Moving Forward")

def move_backward():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.HIGH)
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.HIGH)
    display_status("Moving Backward")

def turn_left():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.HIGH)
    GPIO.output(motor2_in1, GPIO.HIGH)
    GPIO.output(motor2_in2, GPIO.LOW)
    display_status("Turning Left")

def turn_right():
    GPIO.output(motor1_in1, GPIO.HIGH)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.HIGH)
    display_status("Turning Right")

def stop_motors():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.LOW)
    display_status("Stopped")

# Obstacle detection using YOLO
def detect_obstacle_yolo(frame):
    results = model(frame)
    for result in results:
        if len(result.boxes) > 0:
            return True, result  # Obstacle(s) detected
    return False, None

# Previous OpenCV-based detection (commented out)
"""
def detect_obstacle(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, threshold = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return len(contours) > 0
"""
if not cap.isOpened():
    print("❌ Failed to open camera.")
    exit() #This will check whether camera is working

# Main loop
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        obstacle_detected, result = detect_obstacle_yolo(frame)

        if obstacle_detected:
            stop_motors()
            display_status("Object Detected! Avoiding")
            time.sleep(1)
            turn_left()
            time.sleep(1)
        else:
            move_forward()

        # Draw YOLO boxes for visualization
        if result:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow('YOLO Obstacle Avoidance Robot', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    stop_motors()
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
