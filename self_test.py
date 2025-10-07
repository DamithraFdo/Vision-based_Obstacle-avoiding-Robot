#motor_test and motor functions
#in this code we will test the motors and repeat x times

import RPi.GPIO as GPIO
import time 
from config import motor1_in1, motor1_in2, motor2_in1, motor2_in2
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
# Example usage
if __name__ == "__main__":
    try:
        test_motors(duration=2)  # Test each movement for 2 seconds
    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        GPIO.cleanup()

