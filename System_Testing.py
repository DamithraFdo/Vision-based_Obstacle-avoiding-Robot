# System Testing Script for Raspberry Pi Vision Robot
# This script tests the motors, camera, YOLO object detection, and OLED display.
# It provides a menu for selecting individual tests or a full system test.
# It is designed to be run on a Raspberry Pi with the necessary hardware connected.

#-------------------------------- Import Libraries --------------------------------
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from ultralytics import YOLO  # YOLOv8 (recommended)

import board
import busio        
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont

#----------------------------- Motor Setup ---------------------------------------
# Motor control GPIO pins
motor1_in1 = 27
motor1_in2 = 22
motor2_in1 = 23
motor2_in2 = 24

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1_in1, GPIO.OUT)
GPIO.setup(motor1_in2, GPIO.OUT)
GPIO.setup(motor2_in1, GPIO.OUT)
GPIO.setup(motor2_in2, GPIO.OUT)

# Stop motors initially
def stop_motors():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.LOW)

stop_motors()

def move_forward():
    GPIO.output(motor1_in1, GPIO.HIGH)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in1, GPIO.HIGH)
    GPIO.output(motor2_in2, GPIO.LOW)

def move_backward():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.HIGH)
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.HIGH)

def turn_left():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.HIGH)
    GPIO.output(motor2_in1, GPIO.HIGH)
    GPIO.output(motor2_in2, GPIO.LOW)

def turn_right():
    GPIO.output(motor1_in1, GPIO.HIGH)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.HIGH)

def test_motors(duration=2):
    print("Testing motors...")
    move_forward()
    time.sleep(duration)
    stop_motors()
    time.sleep(1)

    move_backward()
    time.sleep(duration)
    stop_motors()
    time.sleep(1)

    turn_left()
    time.sleep(duration)
    stop_motors()
    time.sleep(1)

    turn_right()
    time.sleep(duration)
    stop_motors()
    time.sleep(1)

    print("Motors test complete.")

#----------------------------- OLED Display Setup --------------------------------
i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_128x32(i2c)

def display_status(message):
    # Clear and write message to OLED
    image = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    draw.text((0, 0), message, font=font, fill=255)
    oled.image = image
    oled.show()

def test_oled():
    print("Testing OLED display...")
    display_status("Testing OLED")
    time.sleep(2)
    display_status("OLED Test Complete")
    time.sleep(2)
    oled.fill(0)
    oled.show()
    print("OLED test complete.")

#----------------------------- Camera Test --------------------------------------
def test_camera():
    print("Testing camera...")
    cap = cv2.VideoCapture(0)  # Open default camera
    if not cap.isOpened():
        print("Failed to open camera.")
        return

    for i in range(5):
        ret, frame = cap.read()
        if ret:
            print(f"Displaying frame {i+1}")
            cv2.imshow("Camera Test", frame)
            cv2.waitKey(500)
        else:
            print("Failed to capture frame.")

    cap.release()
    cv2.destroyAllWindows()
    print("Camera test complete.")

#----------------------------- YOLO Object Detection Test ------------------------
model = YOLO('yolov8n.pt')  # Use YOLOv8 model

def test_yolo():
    print("Testing YOLO model...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open camera.")
        return

    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame for YOLO test.")
        cap.release()
        return

    results = model(frame)

    for result in results:
        if len(result.boxes) > 0:
            print(f"Detected {len(result.boxes)} object(s).")
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Optional: label
                if result.names and box.cls is not None:
                    label = result.names[int(box.cls[0])]
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            print("No objects detected.")

    cv2.imshow("YOLO Test", frame)
    cv2.waitKey(3000)
    cv2.destroyAllWindows()
    cap.release()
    print("YOLO test complete.")

#----------------------------- Full System Test ---------------------------------
def test_full_system():
    print("Running full system test...")
    display_status("Full System Test")
    test_camera()
    test_yolo()
    test_motors()
    test_oled()
    display_status("System Test Done")
    print("Full system test complete.")

#----------------------------- Menu and Execution --------------------------------
if __name__ == "__main__":
    try:
        print("\n--- Raspberry Pi Robot Test Menu ---")
        print("1. Test Motors")
        print("2. Test Camera")
        print("3. Test YOLO")
        print("4. Test OLED")
        print("5. Test Full System")

        choice = input("Enter your choice: ")

        if choice == "1":
            test_motors()
        elif choice == "2":
            test_camera()
        elif choice == "3":
            test_yolo()
        elif choice == "4":
            test_oled()
        elif choice == "5":
            test_full_system()
        else:
            print("Invalid choice.")

    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting...")

    finally:
        # Safe shutdown
        stop_motors()
        try:
            cv2.destroyAllWindows()
        except:
            pass
        GPIO.cleanup()
        print("GPIO cleaned up. Exiting.")
        display_status("Goodbye!")  
        # time.sleep(2)
        # oled.fill(0)    
        # oled.show()
        # print("System shutdown complete.")      
# End of System Testing Script
# This script tests the motors, camera, YOLO object detection, and OLED display on a Raspberry Pi robot.
# It provides a menu for selecting individual tests or a full system test.  