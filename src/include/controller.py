import cv2
import lane_detection
import time
import numpy as np
from simple_pid import PID
import RPi.GPIO as GPIO

# Disable warnings
GPIO.setwarnings(False)

# Setup for GPIO pins
in3 = 23
in4 = 24
en2 = 13

in1 = 27
in2 = 17	
en1 = 26

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en1,GPIO.OUT)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en2,GPIO.OUT)

# Initialize the PWM instances for the motors
left_motor = GPIO.PWM(en1, 1000) 
right_motor = GPIO.PWM(en2, 1000) 

# Start the PWM with 0 duty cycle (motor off)
left_motor.start(0)
right_motor.start(0)

# Create PID controller
pid = PID(0.8, 0.6, 0.1)
pid.setpoint = 0.0

# Define the base speed
base_speed = 50 # Adjust this according to your needs

def control_motors(error, curve):
    # Calculate the PID output
    pid_output = pid(error)

    # Reduce speed on curves
    speed_factor = 0.6 if curve else 0.8


    left_motor_speed = base_speed * speed_factor - pid_output

    right_motor_speed = base_speed * speed_factor + pid_output

    # Ensure motor speeds are within acceptable range (0 to 100% duty cycle)
    left_motor_speed = max(0, min(100, left_motor_speed))
    right_motor_speed = max(0, min(100, right_motor_speed))

    left_motor.ChangeDutyCycle(left_motor_speed)
    right_motor.ChangeDutyCycle(right_motor_speed)

    # If motor speeds are zero, turn off the motors and stop PWM
    if left_motor_speed == 0:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        left_motor.stop()
    else:
        left_motor.start(left_motor_speed)
        GPIO.output(in1, GPIO.HIGH if left_motor_speed >= 0 else GPIO.LOW)
        GPIO.output(in2, GPIO.LOW if left_motor_speed >= 0 else GPIO.HIGH)

    if right_motor_speed == 0:
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
        right_motor.stop()
    else:
        right_motor.start(right_motor_speed)
        GPIO.output(in3, GPIO.HIGH if right_motor_speed >= 0 else GPIO.LOW)
        GPIO.output(in4, GPIO.LOW if right_motor_speed >= 0 else GPIO.HIGH)


# Open camera
cap = cv2.VideoCapture(0)

try:
   while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to fetch frame")
        break

    error, curve = lane_detection.process_frame(frame)
    if error > 1000:
        error = 100
    elif error < -1000:
        error = -100
    # Control motors based on error and curve
    control_motors(error, curve)
    
except KeyboardInterrupt:
    print("Interrupted by user")
finally:
    # When everything is done, release the capture and cleanup GPIO
    cap.release()
    left_motor.stop()
    right_motor.stop()
    GPIO.cleanup()