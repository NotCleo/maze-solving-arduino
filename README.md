Maze-Solving Arduino Robot

This project develops an autonomous maze-solving robot using an Arduino Uno, designed to navigate a 4×4 wall maze using two algorithms: Wall Following and Flood Fill. The robot uses three ultrasonic sensors to detect obstacles and employs PWM for efficient motor control. It first navigates using the Wall Following algorithm while mapping the maze, then applies the Flood Fill algorithm to find the shortest path in subsequent runs. The project includes hardware and software development, with performance testing on a 4×4 maze to verify shortest path capability.
Components

    Arduino Uno (ATmega328P microcontroller)
    3 HC-SR04 Ultrasonic Sensors (for distance measurement: front, left, right)
    L298 Dual H-Bridge Motor Driver (dual bidirectional motor control)
    2 DC Gear Motors (for robot propulsion)
    7805 Voltage Regulator IC (provides 5V to ultrasonic sensors)
    Power Supply:
        4 AA batteries (in series, for motor driver)
        2 9V batteries (one for Arduino, one for voltage regulator)

Theory

This robot is designed as a wall maze-solving robot, with its hardware and software tailored for this purpose. The left and right ultrasonic sensors track the left and right walls, while the front sensor maps the front wall. A threshold of 5 cm is set to detect "T-junction" situations, where the robot identifies a junction by checking if the front sensor reading is below this value while the side sensors indicate open paths. Initially, the robot navigates the maze using the Wall Following algorithm. Simultaneously, it constructs a map of the maze using data from all sensors, storing the map values cell by cell in its memory. After the first run, the robot employs the Flood Fill algorithm to determine the shortest path. It retrieves the map values from the Arduino’s memory, converts them into a one-dimensional array, and applies the Flood Fill algorithm to optimize the path.
Algorithm Summaries
Wall Following Algorithm

The Wall Following algorithm, also known as the left-hand or right-hand rule, is a simple maze-solving method. The robot navigates by keeping a wall on its chosen side (left or right). At junctions, it prioritizes turns based on the presence of walls: turning toward an open path while maintaining wall contact. The algorithm ensures the robot reaches the maze’s end but is inefficient for mazes with loops, as it may get trapped in closed regions. The decision rules are:

    Right Wall Following: Turn right if no wall on the right, go straight if no wall ahead, turn left if no wall on the left, otherwise turn around.
    Left Wall Following: Turn left if no wall on the left, go straight if no wall ahead, turn right if no wall on the right, otherwise turn around.

Flood Fill Algorithm

The Flood Fill algorithm, derived from the Bellman-Ford algorithm, is an efficient maze-solving method. It assigns distance values to each cell in the maze, representing the number of steps to the destination cell. Two arrays are used: one for the wall map (indicating walls as bits: North, East, South, West) and another for distance values. The robot updates the wall map, floods the maze with distance values starting from the destination (value 0), and moves to the neighboring cell with the lowest distance value. This ensures the shortest path to the destination. Steps include:

    Update the wall map.
    Flood the maze with distance values.
    Identify the neighboring cell with the lowest distance.
    Move to that cell.

Getting Started

    Assemble the robot by connecting the components as per the pin configuration in the code.
    Upload the maze-solving sketch to the Arduino Uno.
    Place the robot in a 4×4 maze and power it on to begin navigation.

Resources

    Research Reference: IJCA Research Paper
    Arduino Uno Schematic: Arduino Uno Rev3 Schematic (useful for understanding pin assignments and troubleshooting hardware connections)

Possible Hardware Debugging Steps

    Verify Ultrasonic Sensor Wiring: Ensure trigger and echo pins of all HC-SR04 sensors are correctly connected to Arduino and powered via the 7805 regulator (5V).
    Test Motor Functionality: Run a simple sketch to check if both DC gear motors spin correctly in both directions using the L298 driver.
    Monitor Power Supply: Use a multimeter to confirm 4 AA batteries deliver ~6V to the motor driver and each 9V battery powers the Arduino and regulator adequately.
    Check Sensor Readings: Print distance values to Serial Monitor; erratic or zero readings may indicate loose connections or interference.
    Inspect L298 Driver: Ensure no overheating; verify output voltages to motors match PWM signals from Arduino.
