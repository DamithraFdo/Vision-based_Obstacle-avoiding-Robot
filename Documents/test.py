#----------------------------------------- Test Motors --------------------------------------------------------------
import RPi.GPIO as GPIO
import time

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

def move_forward():
    GPIO.output(motor1_in1, GPIO.HIGH)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in1, GPIO.HIGH)
    GPIO.output(motor2_in2, GPIO.LOW)
    # display_status("Moving Forward")

def move_backward():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.HIGH)
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.HIGH)
    # display_status("Moving Backward")

def turn_left():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.HIGH)
    GPIO.output(motor2_in1, GPIO.HIGH)
    GPIO.output(motor2_in2, GPIO.LOW)
    # display_status("Turning Left")

def turn_right():
    GPIO.output(motor1_in1, GPIO.HIGH)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.HIGH)
    # display_status("Turning Right")

def stop_motors():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.LOW)
    # display_status("Stopped")


def test_motors():
    print("Testing motors...")
    move_forward()
    time.sleep(2)
    stop_motors()
    time.sleep(1)

    move_backward()
    time.sleep(2)
    stop_motors()
    time.sleep(1)

    turn_left()
    time.sleep(2)
    stop_motors()
    time.sleep(1)

    turn_right()
    time.sleep(2)
    stop_motors()
    time.sleep(1)

    print("Motors test complete.")

#------------------------------------------- Test Camera ------------------------------------------

import cv2

cap = cv2.VideoCapture(0)

def test_camera():
    print("Testing camera...")
    if not cap.isOpened():
        print("Failed to open camera.")
        return
    
    for i in range(5):  # Capture 5 frames
        ret, frame = cap.read()
        if ret:
            print(f"Displaying frame {i+1}")
            cv2.imshow("Camera Test", frame)
            cv2.waitKey(500)  # Show each frame for 500ms
        else:
            print("Failed to capture frame.")
    
    cv2.destroyAllWindows()
    print("Camera test complete.")

#---------------------------------------- Test YOLO --------------------------------------------------
from ultralytics import YOLO

model = YOLO('yolov5n.pt')

def test_yolo():
    print("Testing YOLO model...")
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame for YOLO test.")
        return
    
    results = model(frame)
    for result in results:
        if len(result.boxes) > 0:
            print(f"Detected {len(result.boxes)} object(s).")
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                print(f"Box coordinates: ({x1}, {y1}), ({x2}, {y2})")
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    cv2.imshow("YOLO Test", frame)
    cv2.waitKey(3000) 
    cv2.destroyAllWindows()
    print("YOLO test complete.")

#------------------------------------ Test OLED -------------------------------------------------
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_128x32(i2c)
oled.fill(0)
oled.show()

def display_status(message):
    oled.fill(0)
    draw = ImageDraw.Draw(oled.image)
    font = ImageFont.load_default()
    draw.text((0, 0), message, font=font, fill=255)
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

#----------------------------------- Full Test ------------------------------------------------

def test_full_system():
    print("Testing full system...")
    display_status("Full System Test")
    test_camera()
    test_yolo()
    test_motors()
    test_oled()
    
    display_status("System Test Complete")
    print("Full system test complete.")

#-----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    try:
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
        print("Exiting tests.")
    finally:
        stop_motors()
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
