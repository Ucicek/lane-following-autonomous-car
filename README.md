# Self-Driving Vehicle Control System
This project implements a self-driving vehicle control system using a combination of C++ and Python. The core of the system involves two primary parts: lane detection and a control system. The lane detection is performed using OpenCV in C++ for real-time image processing, and the control system is implemented in Python, using a PID controller. 
## Table of Contents
- [Overview](#overview)
- [Demo](#demo)
- [Lane Detection](#lane-detection)
- [Control System](#control-system)
- [Running the Project - Software Requirements](#running-the-project---software-requirements)
- [Running the Project - Hardware Requirements](#running-the-project---hardware-requirements)
  

## Overview
This project implements a self-driving vehicle control system using a combination of Python and C++. The core of the system involves two primary parts: lane detection and a control system. The lane detection is performed using OpenCV in C++ for real-time image processing, and the control system is implemented in Python, using a PID controller.

C++ was chosen for the image processing part due to its speed and efficiency. In a real-time application such as this, where decisions need to be made quickly based on the detected lanes, the performance advantage of C++ is significant. The control system, on the other hand, doesn't need to process large amounts of data and doesn't require such high performance, so it's implemented in Python for its simplicity and ease of use.
## Demo


https://github.com/Ucicek/lane-following-autonomous-car/assets/77251886/8bfddb1d-0525-4bfa-8b80-87a6ba41c310



## Lane Detection
The lane detection algorithm is implemented in C++ using the OpenCV library and compiled into a Python extension using pybind11, allowing it to be imported and used in Python scripts.

Here's a brief explanation of how the lane detection algorithm works:

1. **Image Preprocessing**: The input to the function `process_frame` is a frame captured by the vehicle's camera. This frame is first resized to reduce computation. Several image transformations are applied, including color filtering (to isolate lane lines), grayscaling, Gaussian blurring (to reduce noise), and edge detection using the Canny algorithm.
2. **Line Detection**: The Hough transform is used on the processed image to detect lines, which are presumed to be lane lines. The lines are then sorted by their y-coordinates and only the top 10 longest lines are retained to account for curves.
3. **Lane Splitting**: The lines are split into right and left lanes based on their slopes. The slopes and intercepts of the lane lines are computed and used to calculate the midpoint at the bottom of the image.
4. **Error and Curve Determination**: The horizontal distance of this midpoint from the center of the image is calculated and returned as the lane error. Also, the function determines if there's a curve ahead based on the slopes of the lane lines.

The lane detection module exposes a single function, `process_frame`, which can be imported and used in Python.

https://github.com/Ucicek/autonomous_car/assets/77251886/0eb9cf70-939f-48f9-afff-4825edf3d65a



## Control System
The control system is implemented in Python and consists of a PID (Proportional-Integral-Derivative) controller. The controller uses the lane error obtained from the lane detection module to adjust the speeds of the vehicle's left and right motors.

If the vehicle is veering off to the right, the PID controller reduces the speed of the right motor and increases the speed of the left motor, and vice versa.

The PID controller algorithm consists of three separate parameters: the proportional, the integral, and the derivative. These values are "tuned" so that the vehicle can react appropriately to the lane error:

- The **proportional** component produces an output value that is proportional to the current error value. If the error is large, the control output will also be large.
- The **integral** component is proportional to both the magnitude of the error and the duration of the error. The integral response will be small when the error is small.
- The **derivative** component is proportional to the rate of change of the error. If the error is rapidly changing, the derivative response will be large.

The controller also introduces the concept of "sleeping" the motors based on the PID output. If the PID output is too high, indicating a large error, the corresponding motor is stopped for a short period of time, effectively reducing its speed. This helps in correcting the vehicle's path.

**Demo Videos before tunning the PID** 


[Video 1](https://github.com/Ucicek/autonomous_car/assets/77251886/b224acc3-1a3e-47cf-9f6f-27a4d97acc5d) - [Video 2](https://github.com/Ucicek/autonomous_car/assets/77251886/92929591-4b71-4de2-9d60-ccf0d3b67611)




## Running the Project - Software Requirements
1. Compile the C++ module using pybind11.
2. Pip install -r requirements.txt
3. Run the `controller.py` script.

For more information on the PID controller and tuning its parameters, please refer to the comments and documentation within `controller.py`.

This project showcases the power of combining Python and C++ to handle different aspects of a problem, leveraging the strengths of both languages. C++ is used to efficiently process image data in real-time, and Python is used to simply and effectively control the vehicle based on the processed data.

## Running the Project - Hardware Requirements

For this project, you will need the following hardware components:

1. **Raspberry Pi:** The Raspberry Pi serves as the central control unit for the vehicle. It runs the main software and interfaces with the various hardware components. Ensure that your Raspberry Pi is set up with a suitable operating system (like Raspberry Pi OS) and has Python installed.

2. **Raspberry Pi Camera:** This is a key component of the lane detection system. The camera continuously captures frames of the road, which are then processed to identify the lanes. Ensure that the camera is properly connected to the Raspberry Pi.

3. **L298N Motor Driver:** The L298N Motor Driver is used to control the speed and direction of the vehicle's motors. It receives commands from the Raspberry Pi and drives the motors accordingly. Ensure that the L298N is correctly wired to the Raspberry Pi and the vehicle's motors.

4. **Motors:** The vehicle requires motors to move. These motors are controlled by the L298N Motor Driver. The number and type of motors you need will depend on your specific vehicle setup.

5. **Power Bank:** A portable power bank is used to power the Raspberry Pi. Choose a power bank that has enough capacity to power your Raspberry Pi for the duration of your project.

6. **Jumper Wires:** Jumper wires are used to make the necessary electrical connections between the various components. You will need both male-to-female and male-to-male jumper wires for this project.



