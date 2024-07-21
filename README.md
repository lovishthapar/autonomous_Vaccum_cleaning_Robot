vaccum_cleaner.jpeg

# Autonomous Vacuum Cleaning Robot

This project showcases an autonomous vacuum cleaning robot designed and built as part of the Experiential Learning Centre (ELC) activity at Thapar Institute of Engineering & Technology, Patiala. The robot uses an ultrasonic sensor to detect obstacles and navigate through a room, ensuring efficient cleaning.

## Project Overview

- **Project Name**: Autonomous Vacuum Cleaning Robot
- **Institution**: Experiential Learning Centre (ELC), Thapar Institute of Engineering & Technology, Patiala
- **Duration**: Summer Break
- **Objective**: To build an autonomous robot capable of vacuum cleaning by navigating through a room and avoiding obstacles using an ultrasonic sensor.

## Features

- **Autonomous Navigation**: The robot uses an ultrasonic sensor to detect obstacles and navigate through the room.
- **Obstacle Avoidance**: When an obstacle is detected, the robot changes its direction to avoid collision.
- **Efficient Cleaning**: The robot covers the entire room area for effective cleaning.

## Components Used

- **Ultrasonic Sensor**: For obstacle detection and distance measurement.
- **Arduino Mega**: Microcontroller for processing sensor data and controlling the robot's movements.
- **Servo Motor**: To rotate the ultrasonic sensor for better obstacle detection.
- **BTS7960 Motor Drivers**: To control the motors driving the robot.
- **Chassis**: Custom-built body to house all components and provide structural support.
- **BLDC motor**:BLDC motor is atttahced to custom made fan to push the air outside in order to create vaccum,for suction of dust


## How It Works

1. **Initialization**: The robot starts by scanning the environment using the ultrasonic sensor mounted on a servo motor.
2. **Obstacle Detection**: If an obstacle is detected within a certain range, the robot calculates the distance and determines the best direction to move.
3. **Movement Control**: The robot moves in the calculated direction to avoid the obstacle while continuing to clean the room.
4. **Continuous Operation**: This process repeats continuously, allowing the robot to clean the room autonomously.

