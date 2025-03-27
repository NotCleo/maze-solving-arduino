//ultrasonic
#define TRIGGER_FRENTE A4 // Front sensor trigger
#define ECHO_FRENTE A5    // Front sensor echo
#define TRIGGER_ESQ A2    // Left sensor trigger
#define ECHO_ESQ A3       // Left sensor echo
#define TRIGGER_DIR A0    // Right sensor trigger
#define ECHO_DIR A1       // Right sensor echo

//motors
#define VEL_MOTOR_ESQ 10  // Left motor speed (PWM)
#define VEL_MOTOR_DIR 11  // Right motor speed (PWM)
#define E1 8              // Left motor direction 1
#define E2 9              // Left motor direction 2
#define D1 12             // Right motor direction 1
#define D2 7              // Right motor direction 2

#define MAZE_SIZE 4  // Set to desired size (4 for 4x4, 10 for 10x10)

#define T_JUNCTION_THRESHOLD 5  // Threshold for detecting T-junctions
#define WALL_FOLLOW_MIN 7       // Minimum distance for wall following
#define WALL_FOLLOW_MAX 13      // Maximum distance for wall following
#define FRONT_OBSTACLE 8        // Front obstacle detection threshold
#define SIDE_OBSTACLE 20        // Side obstacle detection threshold

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

#define WALL_NORTH 0b0001
#define WALL_EAST  0b0010
#define WALL_SOUTH 0b0100
#define WALL_WEST  0b1000

uint8_t wallMap[MAZE_SIZE][MAZE_SIZE];    // Stores walls (N, E, S, W as bits)
uint8_t distanceMap[MAZE_SIZE][MAZE_SIZE]; // Stores distance values for Flood Fill
bool visited[MAZE_SIZE][MAZE_SIZE];        // Tracks visited cells

// Robot state
int robotX = 0;           // Robot's X position
int robotY = 0;           // Robot's Y position
int orientation = NORTH;  // Robot's orientation (0=North, 1=East, 2=South, 3=West)
bool firstRun = true;     // Flag for first run (Wall Following)
bool reachedEnd = false;  // Flag for reaching the destination

void setup() {
    Serial.begin(9600);

    pinMode(TRIGGER_FRENTE, OUTPUT);
    pinMode(ECHO_FRENTE, INPUT);
    pinMode(TRIGGER_ESQ, OUTPUT);
    pinMode(ECHO_ESQ, INPUT);
    pinMode(TRIGGER_DIR, OUTPUT);
    pinMode(ECHO_DIR, INPUT);

    pinMode(VEL_MOTOR_ESQ, OUTPUT);
    pinMode(VEL_MOTOR_DIR, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);

    for (int i = 0; i < MAZE_SIZE; i++) 
    {
        for (int j = 0; j < MAZE_SIZE; j++) 
        {
            wallMap[i][j] = 0;      // No walls initially
            distanceMap[i][j] = 255; // Unknown distance
            visited[i][j] = false;
        }
    }
    distanceMap[MAZE_SIZE-1][MAZE_SIZE-1] = 0; // Destination has distance 0
    visited[0][0] = true;

    delay(5000); // Wait for initialization
    Serial.println("Starting maze solving...");
}

void loop() {
    if (reachedEnd && !firstRun) 
    {
        // After first run, use Flood Fill to navigate shortest path
        navigateWithFloodFill();
    } 
    else 
    {
        // First run: Use Wall Following and map the maze
        navigateWithWallFollowing();
    }
}

// Read distance from an ultrasonic sensor
int readDistance(int triggerPin, int echoPin) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    return duration / 29 / 2; // Convert to cm
}

// Get sensor readings based on orientation
void getSensorReadings(int &front, int &left, int &right) {
    int rawFront = readDistance(TRIGGER_FRENTE, ECHO_FRENTE);
    int rawLeft = readDistance(TRIGGER_ESQ, ECHO_ESQ);
    int rawRight = readDistance(TRIGGER_DIR, ECHO_DIR);

    front = rawFront;
    left = rawLeft;
    right = rawRight;

    Serial.print("Front: "); Serial.print(front);
    Serial.print(" Left: "); Serial.print(left);
    Serial.print(" Right: "); Serial.print(right);
    Serial.println(" cm");
}

// Update the wall map based on sensor readings
void updateWallMap(int front, int left, int right) {
    uint8_t walls = 0;
    int frontWall, leftWall, rightWall;

    // Determine wall presence based on thresholds
    frontWall = (front <= T_JUNCTION_THRESHOLD) ? 1 : 0;
    leftWall = (left <= SIDE_OBSTACLE) ? 1 : 0;
    rightWall = (right <= SIDE_OBSTACLE) ? 1 : 0;

    // Map walls based on orientation
    switch (orientation) {
        case NORTH:
            if (frontWall) walls |= WALL_NORTH;
            if (leftWall) walls |= WALL_WEST;
            if (rightWall) walls |= WALL_EAST;
            break;
        case EAST:
            if (frontWall) walls |= WALL_EAST;
            if (leftWall) walls |= WALL_NORTH;
            if (rightWall) walls |= WALL_SOUTH;
            break;
        case SOUTH:
            if (frontWall) walls |= WALL_SOUTH;
            if (leftWall) walls |= WALL_EAST;
            if (rightWall) walls |= WALL_WEST;
            break;
        case WEST:
            if (frontWall) walls |= WALL_WEST;
            if (leftWall) walls |= WALL_SOUTH;
            if (rightWall) walls |= WALL_NORTH;
            break;
    }

    wallMap[robotX][robotY] = walls;
}

// Motor control functions
void stopMotors() {
    analogWrite(VEL_MOTOR_ESQ, 0);
    analogWrite(VEL_MOTOR_DIR, 0);
    digitalWrite(E1, LOW);
    digitalWrite(E2, LOW);
    digitalWrite(D1, LOW);
    digitalWrite(D2, LOW);
}

void moveForward() {
    Serial.println("Moving forward");
    analogWrite(VEL_MOTOR_ESQ, 120);
    analogWrite(VEL_MOTOR_DIR, 120);
    digitalWrite(E1, HIGH);
    digitalWrite(E2, LOW);
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
    delay(500); // Adjust delay for cell size
    stopMotors();

    // Update position based on orientation
    switch (orientation) {
        case NORTH: robotY++; break;
        case EAST: robotX++; break;
        case SOUTH: robotY--; break;
        case WEST: robotX--; break;
    }
    visited[robotX][robotY] = true;
}

void turnLeft() {
    Serial.println("Turning left");
    analogWrite(VEL_MOTOR_ESQ, 120);
    analogWrite(VEL_MOTOR_DIR, 120);
    digitalWrite(E1, LOW);
    digitalWrite(E2, HIGH);
    digitalWrite(D1, LOW);
    digitalWrite(D2, HIGH);
    delay(700);
    stopMotors();

    // Update orientation
    orientation = (orientation + 3) % 4; // Turn left: NORTH -> WEST -> SOUTH -> EAST
}

void turnRight() {
    Serial.println("Turning right");
    analogWrite(VEL_MOTOR_ESQ, 120);
    analogWrite(VEL_MOTOR_DIR, 120);
    digitalWrite(E1, HIGH);
    digitalWrite(E2, LOW);
    digitalWrite(D1, HIGH);
    digitalWrite(D2, LOW);
    delay(800);
    stopMotors();

    // Update orientation
    orientation = (orientation + 1) % 4; // Turn right: NORTH -> EAST -> SOUTH -> WEST
}

void turnAround() {
    Serial.println("Turning around");
    turnRight();
    turnRight();
}

// Wall Following navigation
void navigateWithWallFollowing() {
    int front, left, right;
    getSensorReadings(front, left, right);

    // Update the wall map
    updateWallMap(front, left, right);

    // Check if reached the destination
    if (robotX == MAZE_SIZE-1 && robotY == MAZE_SIZE-1) {
        Serial.println("Reached destination!");
        reachedEnd = true;
        firstRun = false;
        stopMotors();
        floodFill(); // Prepare for shortest path navigation
        return;
    }

    // Right wall following logic
    if (right > WALL_FOLLOW_MAX) {
        // No wall on the right, turn right
        turnRight();
        moveForward();
    } else if (front > FRONT_OBSTACLE) {
        // No wall ahead, go straight
        if (right >= WALL_FOLLOW_MIN && right <= WALL_FOLLOW_MAX) {
            // Adjust speed to maintain distance from right wall
            analogWrite(VEL_MOTOR_ESQ, 120);
            analogWrite(VEL_MOTOR_DIR, 150);
            digitalWrite(E1, HIGH);
            digitalWrite(E2, LOW);
            digitalWrite(D1, LOW);
            digitalWrite(D2, HIGH);
            delay(500);
            stopMotors();
            switch (orientation) {
                case NORTH: robotY++; break;
                case EAST: robotX++; break;
                case SOUTH: robotY--; break;
                case WEST: robotX--; break;
            }
            visited[robotX][robotY] = true;
        } else if (right > WALL_FOLLOW_MAX) {
            // Too far from right wall, turn right
            turnRight();
            moveForward();
        } else if (right < WALL_FOLLOW_MIN) {
            // Too close to right wall, turn left
            turnLeft();
            moveForward();
        }
    } else if (left > SIDE_OBSTACLE) {
        // No wall on the left, turn left
        turnLeft();
        moveForward();
    } else {
        // Wall ahead, left, and right: turn around
        turnAround();
    }
}

// Flood Fill algorithm to compute distances
void floodFill() {
    // Reset distance map
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            distanceMap[i][j] = 255;
        }
    }
    distanceMap[MAZE_SIZE-1][MAZE_SIZE-1] = 0; // Destination

    // Use a queue for Flood Fill
    int queueX[MAZE_SIZE * MAZE_SIZE];
    int queueY[MAZE_SIZE * MAZE_SIZE];
    int front = 0, rear = 0;

    queueX[rear] = MAZE_SIZE-1;
    queueY[rear] = MAZE_SIZE-1;
    rear++;

    while (front < rear) {
        int x = queueX[front];
        int y = queueY[front];
        front++;

        int dist = distanceMap[x][y] + 1;

        // Check all four directions
        // North
        if (y + 1 < MAZE_SIZE && !(wallMap[x][y] & WALL_NORTH) && distanceMap[x][y+1] == 255) {
            distanceMap[x][y+1] = dist;
            queueX[rear] = x;
            queueY[rear] = y+1;
            rear++;
        }
        // East
        if (x + 1 < MAZE_SIZE && !(wallMap[x][y] & WALL_EAST) && distanceMap[x+1][y] == 255) {
            distanceMap[x+1][y] = dist;
            queueX[rear] = x+1;
            queueY[rear] = y;
            rear++;
        }
        // South
        if (y - 1 >= 0 && !(wallMap[x][y] & WALL_SOUTH) && distanceMap[x][y-1] == 255) {
            distanceMap[x][y-1] = dist;
            queueX[rear] = x;
            queueY[rear] = y-1;
            rear++;
        }
        // West
        if (x - 1 >= 0 && !(wallMap[x][y] & WALL_WEST) && distanceMap[x-1][y] == 255) {
            distanceMap[x-1][y] = dist;
            queueX[rear] = x-1;
            queueY[rear] = y;
            rear++;
        }
    }

    // Print distance map for debugging
    Serial.println("Distance Map:");
    for (int y = MAZE_SIZE-1; y >= 0; y--) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            Serial.print(distanceMap[x][y]);
            Serial.print(" ");
        }
        Serial.println();
    }
}

// Navigate using Flood Fill (shortest path)
void navigateWithFloodFill() {
    // Find the neighbor with the lowest distance
    int minDist = 255;
    int nextX = robotX;
    int nextY = robotY;
    int nextDir = orientation;

    // Check all four directions
    if (robotY + 1 < MAZE_SIZE && !(wallMap[robotX][robotY] & WALL_NORTH) && distanceMap[robotX][robotY+1] < minDist) {
        minDist = distanceMap[robotX][robotY+1];
        nextX = robotX;
        nextY = robotY + 1;
        nextDir = NORTH;
    }
    if (robotX + 1 < MAZE_SIZE && !(wallMap[robotX][robotY] & WALL_EAST) && distanceMap[robotX+1][robotY] < minDist) {
        minDist = distanceMap[robotX+1][robotY];
        nextX = robotX + 1;
        nextY = robotY;
        nextDir = EAST;
    }
    if (robotY - 1 >= 0 && !(wallMap[robotX][robotY] & WALL_SOUTH) && distanceMap[robotX][robotY-1] < minDist) {
        minDist = distanceMap[robotX][robotY-1];
        nextX = robotX;
        nextY = robotY - 1;
        nextDir = SOUTH;
    }
    if (robotX - 1 >= 0 && !(wallMap[robotX][robotY] & WALL_WEST) && distanceMap[robotX-1][robotY] < minDist) {
        minDist = distanceMap[robotX-1][robotY];
        nextX = robotX - 1;
        nextY = robotY;
        nextDir = WEST;
    }

    // Turn to face the next direction
    while (orientation != nextDir) {
        if ((orientation + 1) % 4 == nextDir) {
            turnRight();
        } else if ((orientation + 3) % 4 == nextDir) {
            turnLeft();
        } else {
            turnAround();
        }
    }

    // Move to the next cell
    moveForward();
    robotX = nextX;
    robotY = nextY;

    // Check if reached the start (0, 0) again
    if (robotX == 0 && robotY == 0) {
        Serial.println("Returned to start with shortest path!");
        stopMotors();
        while (true); // Stop the robot
    }
}
