# Maze Solving Bot via Arduino

This project implements a maze-solving robot using an Arduino microcontroller and three ultrasonic sensors. The robot navigates through a maze by detecting obstacles in front, left, and right directions, employing a simple wall-following algorithm to find its way out.
Features

    Uses 3 ultrasonic sensors for obstacle detection (front, left, right).
    Controls two DC motors for movement.
    Implements a basic maze-solving logic (e.g., left-hand rule).

Hardware Requirements

    Arduino board (e.g., Uno)
    3 HC-SR04 ultrasonic sensors
    Motor driver (e.g., L298N)
    2 DC motors
    Chassis, power supply, and jumper wires

Getting Started

    Wire the ultrasonic sensors and motors to the Arduino as per the pin definitions in the code.
    Upload the sketch from maze_solving.ino to your Arduino.
    Place the robot in a maze and power it on to start navigation.
    
To-Do List

    [ ]
    [ ]
    [ ]
    [ ]
    [ ]

Possible Hardware Debugging Steps

    Check Sensor Connections: Ensure trigger and echo pins of all 3 ultrasonic sensors are correctly wired and powered (5V and GND).
    Test Motor Operation: Verify motors spin in both directions by manually setting PWM pins high in a test sketch.
    Validate Sensor Readings: Use Serial Monitor to print distance values from each sensor; check for 0 or erratic readings indicating wiring or interference issues.
    Power Supply Check: Confirm the battery or power source provides sufficient voltage/current for both motors and sensors (e.g., use a separate supply for motors if needed).
    Inspect Motor Driver: Ensure the motor driver is properly connected and not overheating; test with a multimeter for correct voltage outputs.
