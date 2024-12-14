include <Servo.h> // Include the Servo library to control servo motors

// Create an instance of the Servo class called Myservo
Servo Myservo; 

// Define the pins for the ultrasonic sensor using descriptive names
#define TRIG_PIN 9           // The trigger pin that initiates the ultrasonic pulse (output)
#define ECHO_PIN 8           // The echo pin that receives the reflected pulse (input)

// Define pins for controlling the left and right motors
#define LEFT_MOTOR_A_PIN 4   // Left motor control pin 1 (controls rotation direction)
#define LEFT_MOTOR_B_PIN 5   // Left motor control pin 2 (controls speed)
#define RIGHT_MOTOR_A_PIN 6  // Right motor control pin 1 (controls rotation direction)
#define RIGHT_MOTOR_B_PIN 7  // Right motor control pin 2 (controls speed)

long duration; // Variable to store the time taken by the echo signal to return
long distance; // Variable to store the calculated distance based on the echo signal

void setup() {
   Serial.begin(9600); // Start serial communication at 9600 baud rate for debugging
    // Set motor control pins as outputs
    pinMode(LEFT_MOTOR_A_PIN, OUTPUT);  
    pinMode(LEFT_MOTOR_B_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_A_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_B_PIN, OUTPUT);
    // Set the trigger pin for the ultrasonic sensor as output
    pinMode(TRIG_PIN, OUTPUT); 
    // Set the echo pin for the ultrasonic sensor as input
    pinMode(ECHO_PIN, INPUT); 
    Myservo.attach(10); // Attach the servo motor to pin 10
}

void loop() {
    // Clear the TRIG_PIN to ensure a clean start; set it LOW for 2 microseconds
    digitalWrite(TRIG_PIN, LOW); 
    delayMicroseconds(2); 

    // Trigger the ultrasonic sensor by sending a HIGH signal for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);       // The pulse must be HIGH for 10 microseconds
    digitalWrite(TRIG_PIN, LOW);  // Turn the trigger pin LOW again

    // Measure the duration it takes for the echo to return
    duration = pulseIn(ECHO_PIN, HIGH); // This will block until the HIGH signal is received

    // Calculate the distance based on the duration of the pulse (speed of sound)
    // The speed of sound is approximately 340m/s or 58.2 microseconds per cm
    distance = duration / 58.2; 
    Serial.print("Distance: "); // Print "Distance:" label to indicate the following value
    Serial.println(distance);    // Output the calculated distance to the Serial Monitor 
    delay(100);                  // Brief delay to avoid flooding the Serial Monitor


    // Check if the measured distance is greater than 2 cm, indicating no obstacles nearby
    if (distance > 2) 
        Myservo.write(90);        // Set the servo to the forward position (90 degrees)
        moveForward();            // Call the function to move the robot forward
    }
    // If an obstacle is detected closer than 2 cm (but greater than 0)
    else if (distance > 0) {
        stopMotors();             // Immediately stop the motors to prevent collision
        delay(100); // Brief pause to allow stabilization after stopping

        // Scan for obstacles by moving the servo side to side
        scanForObstacles();
        
        // Move backward to prevent collision with the detected obstacle
        moveBackward();
        
        // Perform a left turn to change the robot's direction
        turnLeft();
    } 
    // If the distance is measured as 0, which indicates a potential sensor issue
    else {
        Serial.println("Invalid distance detected, adjust the sensor."); // Print error message indicating invalid reading
        stopMotors(); // Stop the motors to ensure safety
    }
}

// Function to move the robot forward
void moveForward() {
    digitalWrite(RIGHT_MOTOR_B_PIN, HIGH);    // Activate the right motor to move forward
    digitalWrite(RIGHT_MOTOR_A_PIN, LOW);      // Set direction of right motor
    digitalWrite(LEFT_MOTOR_B_PIN, HIGH);     // Activate the left motor to move forward
    digitalWrite(LEFT_MOTOR_A_PIN, LOW);       // Set direction of left motor
    Serial.println("Moving forward."); // Indicate that the robot is moving forward
}

// Function to stop all motors to halt movement
void stopMotors() {
    // Set all motor control pins to LOW to stop the motors
    digitalWrite(RIGHT_MOTOR_B_PIN, LOW);      // Stop the right motor
    digitalWrite(RIGHT_MOTOR_A_PIN, LOW); 
    digitalWrite(LEFT_MOTOR_B_PIN, LOW);       // Stop the left motor
    digitalWrite(LEFT_MOTOR_A_PIN, LOW); 
    Serial.println("Motors stopped."); // Indicate that the motors have stopped
}

// Function to move the robot backward for a short duration
void moveBackward() {
    digitalWrite(RIGHT_MOTOR_B_PIN, LOW);      // Stop the right motor
    digitalWrite(RIGHT_MOTOR_A_PIN, HIGH);     // Activate the right motor in reverse
    digitalWrite(LEFT_MOTOR_B_PIN, LOW); 
    digitalWrite(LEFT_MOTOR_A_PIN, HIGH);      // Activate the left motor in reverse
    delay(1000); // Move backward for 1 second
    stopMotors(); // Stop after moving backward
    Serial.println("Moving backward."); // Indicate that the robot is moving backward
}

// Function for scanning the surroundings by moving the servo side-to-side
void scanForObstacles() {
    Myservo.write(0);             // Move the servo to the left (0 degrees) to scan the area
    delay(500);                   // Wait for the servo to reach the leftmost position
    Myservo.write(180);           // Move the servo to the right (180 degrees) to scan the maximum area
    delay(500);                   // Wait for the servo to reach the rightmost position
    Myservo.write(90);            // Reset the servo to the center position (90 degrees)
    delay(500);                   // Wait for servo to stabilize in the center
    Serial.println("Scanning for obstacles."); // Print a message indicating the scanning action
}

// Function to rotate the robot to the left
void turnLeft() {
   digitalWrite(RIGHT_MOTOR_B_PIN, HIGH);     // Activate the right motor to rotate the robot left
   delay(500); // Turn for half a second to complete the left turn
   stopMotors(); // Stop the motors after turning
   Serial.println("Turning left."); // Print a message indicating the robot is turning left
}
