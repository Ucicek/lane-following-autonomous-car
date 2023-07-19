import time
import PID
import RPi.GPIO as GPIO


left_motor_pin = 2
right_motor_pin = 3
GPIO.setmode(GPIO.BCM)
GPIO.setup(left_motor_pin, GPIO.OUT)
GPIO.setup(right_motor_pin, GPIO.OUT)

# Set up PID controller
pid = PID.PID(0.2, 0.0, 0.0)  # PID parameters: Kp, Ki, Kd
pid.SetPoint = 0.0  # The target point is the center of the image
pid.setSampleTime(0.1)  # Set the PID sample time

# Assume a base speed, and that your motor controller accepts speed commands as a duty cycle percentage
base_speed = 70  # A duty cycle percentage. Adjust based on your specific motors.

def control_motors(error):
    # Calculate the PID output
    pid.update(error)
    pid_output = pid.output

    # Calculate left and right motor speeds
    left_motor_speed = base_speed - pid_output
    right_motor_speed = base_speed + pid_output

    # Ensure motor speeds are within acceptable range (0 to 100% duty cycle)
    left_motor_speed = max(0, min(100, left_motor_speed))
    right_motor_speed = max(0, min(100, right_motor_speed))

    # Send speed commands to motors
    GPIO.PWM(left_motor_pin, left_motor_speed)
    GPIO.PWM(right_motor_pin, right_motor_speed)

# Example usage:
while True:
    error = get_lane_error()  # This is your function that communicates with the C++ program and gets the current error
    control_motors(error)
    time.sleep(0.1)  # Sleep for the PID sample time before the next loop
x