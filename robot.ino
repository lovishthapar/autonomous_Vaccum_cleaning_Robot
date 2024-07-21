#include <Servo.h>
#include <IRremote.h>

// Motor driver pins for BTS7960 (main drive motors)
// Left Motor
const int LEFT_MOTOR_LPWM = 2;  // PWM pin
const int LEFT_MOTOR_RPWM = 3;  // PWM pin
const int LEFT_MOTOR_L_EN = 22; // Digital pin
const int LEFT_MOTOR_R_EN = 23; // Digital pin
// const int LEFT_L_IS = 52;
// const int LEFT_R_IS = 53;

// Right Motor
const int RIGHT_MOTOR_LPWM = 4;  // PWM pin
const int RIGHT_MOTOR_RPWM = 5;  // PWM pin
const int RIGHT_MOTOR_L_EN = 24; // Digital pin
const int RIGHT_MOTOR_R_EN = 25; // Digital pin
// const int RIGHT_L_IS = 51;
// const int RIGHT_R_IS = 50;

// L298N motor driver pins (for brush motors)
// const int BRUSH_ENA = 8;  // PWM pin
// const int BRUSH_ENB = 9;  // PWM pin
const int BRUSH_IN1 = 30; // Digital pin
const int BRUSH_IN2 = 31; // Digital pin
const int BRUSH_IN3 = 32; // Digital pin
const int BRUSH_IN4 = 33; // Digital pin

// Ultrasonic sensor pins
const int TRIGGER_PIN_FRONT = 41;  // Digital pin
const int ECHO_PIN_FRONT = 43;     // Digital pin
const int TRIGGER_PIN_BOTTOM = 34; // Digital pin
const int ECHO_PIN_BOTTOM = 35;    // Digital pin

// Servo motor pin
const int SERVO_PIN = 6; // PWM pin

// ESC (Electronic Speed Controller) pin for vacuum
const int ESC_PIN = 7; // PWM pin

// IR receiver pin
const int IR_RECEIVER_PIN = 12; // Digital pin

// LED pin
const int LED_PIN = 13; // Digital pin

// IR remote button codes (you may need to adjust these based on your specific remote)
const long IR_BUTTON_POWER = 0xFF10EF;
const long IR_BUTTON_1 = 0x01FEF807;
const long IR_BUTTON_2 = 0xFF130EF;
const long IR_BUTTON_3 = 0xFF11EF;
const long IR_BUTTON_4 = 0xFF12EF;
const long IR_BUTTON_5 = 0xFF38C7;
const long IR_BUTTON_6 = 0xFF5AA5;
const long IR_BUTTON_7 = 0xFF42BD;
const long IR_BUTTON_8 = 0xFF4AB5;
const long IR_BUTTON_9 = 0xFF52AD;

IRrecv irReceiver(IR_RECEIVER_PIN);
decode_results irResults;

Servo myServo;
const int SCAN_ANGLES[] = {0, 45, 90, 135, 180}; // Angles to scan
const int NUM_SCAN_ANGLES = 5;
const int MIN_CLEAR_DISTANCE = 15;

// Timing variables
unsigned long lastMoveTime = 0;
const unsigned long MOVE_INTERVAL = 50; // Update movement every 50ms

// Zigzag pattern variables
bool movingRight = true;
int zigzagWidth = 200; // Initial width of zigzag pattern
const int MIN_ZIGZAG_WIDTH = 150;
const int MAX_ZIGZAG_WIDTH = 300;
const int ZIGZAG_WIDTH_STEP = 50;

// Coverage tracking
const int GRID_SIZE = 1000; // cm
const int MAX_GRID_X = 75;
const int MAX_GRID_Y = 75;
const int TOTAL_CELLS = MAX_GRID_X * MAX_GRID_Y;
uint8_t coverageGrid[TOTAL_CELLS / 8 + 1]; // Each byte stores 8 cells
int currentX = 0;
int currentY = 0;
int direction = 0; // 0: North, 1: East, 2: South, 3: West

// Robot state
bool isRobotOn = true;

// Motor and sensor states
bool leftMotorOn = true;
bool rightMotorOn = true;
bool brushMotorOn = true;
bool vacuumMotorOn = true;
bool frontSensorOn = true;
bool bottomSensorOn = false;
bool servoMotorOn = true;

// Error handling variables
bool sensorError = false;
bool motorError = false;
unsigned long lastErrorTime = 0;
const unsigned long ERROR_DISPLAY_INTERVAL = 5000; // Display error every 5 seconds

void setup()
{
    Serial.begin(9600);
    myServo.attach(SERVO_PIN);
    myServo.write(90); // Set servo to look forward

    // Initialize IR receiver
    irReceiver.enableIRIn();

    // Set motor driver pins as outputs
    pinMode(LEFT_MOTOR_LPWM, OUTPUT);
    pinMode(LEFT_MOTOR_RPWM, OUTPUT);
    pinMode(LEFT_MOTOR_L_EN, OUTPUT);
    pinMode(LEFT_MOTOR_R_EN, OUTPUT);
    pinMode(RIGHT_MOTOR_LPWM, OUTPUT);
    pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
    pinMode(RIGHT_MOTOR_L_EN, OUTPUT);
    pinMode(RIGHT_MOTOR_R_EN, OUTPUT);

    // Set L298N pins as outputs
    // pinMode(BRUSH_ENA, OUTPUT);
    // pinMode(BRUSH_ENB, OUTPUT);
    pinMode(BRUSH_IN1, OUTPUT);
    pinMode(BRUSH_IN2, OUTPUT);
    pinMode(BRUSH_IN3, OUTPUT);
    pinMode(BRUSH_IN4, OUTPUT);

    // Set ultrasonic sensor pins
    pinMode(TRIGGER_PIN_FRONT, OUTPUT);
    pinMode(ECHO_PIN_FRONT, INPUT);
    pinMode(TRIGGER_PIN_BOTTOM, OUTPUT);
    pinMode(ECHO_PIN_BOTTOM, INPUT);

    // Set ESC pin
    pinMode(ESC_PIN, OUTPUT);
    analogWrite(ESC_PIN, 10); // Initialize ESC to 0 speed

    // Set LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // LED off (Robot is off)

    Serial.println("Setup complete. Robot is ready.");
    printRobotStatus();
}

void loop() {
  if (irReceiver.decode(&irResults)) {
    handleIRCommand(irResults.value);
    irReceiver.resume();
    blinkLED();
  }

  if (isRobotOn) {
    digitalWrite(LED_PIN, HIGH);

    unsigned long currentTime = millis();

    // Operate continuous components (brush motor and vacuum)
    operateContinuousComponents();

    // Check ultrasonic sensors
    int frontDistance = measureDistance(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT);
    int bottomDistance = measureDistance(TRIGGER_PIN_BOTTOM, ECHO_PIN_BOTTOM);

    if (frontDistance == -1 && frontSensorOn) {
      reportSensorError("Front");
    }
    if (bottomDistance == -1 && bottomSensorOn) {
      reportSensorError("Bottom");
    }

    //updatePosition();
    //markCurrentPositionCleaned();
  
    // Only move the robot if there's a clear path
    if (frontDistance > MIN_CLEAR_DISTANCE && (!bottomSensorOn || (bottomDistance > 0 && bottomDistance <= 5))) {
      if (currentTime - lastMoveTime >= MOVE_INTERVAL) {
        updateZigzagMovement();
        lastMoveTime = currentTime;
      } else {
        moveForward();
      }
    } else {
      stopMainMotors();
      if (frontDistance > 0 && frontDistance <= MIN_CLEAR_DISTANCE && frontSensorOn) {
        handleFrontObstacle(frontDistance);
      } else if (bottomDistance > 5 && bottomSensorOn) {
        handleStepOrDrop();
      }
    }
    
    adjustZigzagWidth();
    
    if (isGridComplete()) {
      resetGrid();
      Serial.println("Grid coverage complete. Resetting grid.");
    }

    // Periodic motor check (every 5 seconds)
    static unsigned long lastMotorCheckTime = 0;
    if (currentTime - lastMotorCheckTime >= 5000) {
      checkMotorFunctionality();
      lastMotorCheckTime = currentTime;
    }
  } else {
    stopAllMotors();
    digitalWrite(LED_PIN, LOW);
  }

  displayActiveErrors();
}

void operateContinuousComponents() {
  // Operate brush motor continuously
  if (brushMotorOn) {
    if (!operateBrushMotor()) {
      reportMotorError("Brush");
    }
  } else {
    stopBrushMotor();
  }

  // Operate vacuum motor continuously
  if (vacuumMotorOn) {
    if (!operateVacuumMotor()) {
      reportMotorError("Vacuum");
    }
  } else {
    stopVacuumMotor();
  }
}

void stopMainMotors() {
  stopLeftMotor();
  stopRightMotor();
}

void updateZigzagMovement()
{
    static bool isturning = false;

    if (isturning)
    {
        if (movingRight)
        {
            turnRight();
        }
        else
        {
            turnLeft();
        }
        delay(calculateTurnTime());
        isturning = false;
        movingRight = !movingRight;
    }
    else
    {
        moveForward();
        delay(zigzagWidth * 10); // Move forward for a distance proportional to zigzagWidth
        isturning = true;
    }
}

void checkMotorFunctionality()
{
    bool allMotorsOK = true;

    // Check left main motor
    if (leftMotorOn)
    {
        if (!operateLeftMotor())
        {
            reportMotorError("Left main");
            allMotorsOK = false;
        }
    }

    // Check right main motor
    if (rightMotorOn)
    {
        if (!operateRightMotor())
        {
            reportMotorError("Right main");
            allMotorsOK = false;
        }
    }

    // Check brush motor
    if (brushMotorOn)
    {
        if (!operateBrushMotor())
        {
            reportMotorError("Brush");
            allMotorsOK = false;
        }
    }

    // Check vacuum motor
    if (vacuumMotorOn)
    {
        if (!operateVacuumMotor())
        {
            reportMotorError("Vacuum");
            allMotorsOK = false;
        }
    }

    // Check servo motor
    if (servoMotorOn)
    {
        if (!checkServoMotor())
        {
            reportMotorError("Servo");
            allMotorsOK = false;
        }
    }

    if (allMotorsOK)
    {
        Serial.println("All motors are functioning correctly.");
    }
    else
    {
        Serial.println("One or more motors are not functioning correctly. Check error messages.");
    }
}

bool checkServoMotor()
{
    int initialPosition = myServo.read();
    int testPosition = (initialPosition + 30) % 180; // Move 30 degrees, wrapping around at 180

    myServo.write(testPosition);
    delay(500); // Give the servo time to move

    int newPosition = myServo.read();
    myServo.write(initialPosition); // Return to initial position

    return abs(newPosition - testPosition) < 5; // Allow for small measurement error
}

void handleFrontObstacle(int distance) {
    stopAllMotors();
    delay(500);

    int clearAngle = scanForClearPath();

    if (clearAngle != -1) {
        // Clear path found, turn to that angle
        turnToAngle(clearAngle);
        
        // Double-check the path is clear after turning
        int newDistance = measureDistance(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT);
        if (newDistance > MIN_CLEAR_DISTANCE) {
            // Path is clear, move forward
            moveForward();
            return;
        }
    }

    // If we're here, either no clear path was found or the path wasn't clear after turning
    reverseAndTurn();
}

int scanForClearPath() {
    if (!servoMotorOn) {
        return -1; // Cannot scan if servo is off
    }

    int maxDistance = 0;
    int bestAngle = -1;

    // Scan from left to right
    for (int angle = 0; angle <= 180; angle += 45) {
        myServo.write(angle);
        delay(500); // Wait for servo to reach position

        int distance = measureDistance(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT);

        if (distance > maxDistance && distance >= MIN_CLEAR_DISTANCE) {
            maxDistance = distance;
            bestAngle = angle;
        }

        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.print(", Distance: ");
        Serial.println(distance);
    }

    myServo.write(90); // Return to forward position
    return bestAngle;
}

void turnToAngle(int angle)
{
    int currentAngle = 90; // Assume starting from forward position
    int turnAmount = angle - currentAngle;

    if (turnAmount > 0)
    {
        turnRight();
    }
    else
    {
        turnLeft();
    }

    delay(abs(turnAmount) * 100); // Adjust this multiplier based on your robot's turning speed
    stopAllMotors();
}

void turnLeft()
{
    digitalWrite(LEFT_MOTOR_L_EN, HIGH);
    digitalWrite(LEFT_MOTOR_R_EN, HIGH);
    analogWrite(LEFT_MOTOR_LPWM, 0);
    analogWrite(LEFT_MOTOR_RPWM, 200);

    digitalWrite(RIGHT_MOTOR_L_EN, HIGH);
    digitalWrite(RIGHT_MOTOR_R_EN, HIGH);
    analogWrite(RIGHT_MOTOR_LPWM, 200);
    analogWrite(RIGHT_MOTOR_RPWM, 0);
}

void turnRight()
{
    digitalWrite(LEFT_MOTOR_L_EN, HIGH);
    digitalWrite(LEFT_MOTOR_R_EN, HIGH);
    analogWrite(LEFT_MOTOR_LPWM, 200);
    analogWrite(LEFT_MOTOR_RPWM, 0);

    digitalWrite(RIGHT_MOTOR_L_EN, HIGH);
    digitalWrite(RIGHT_MOTOR_R_EN, HIGH);
    analogWrite(RIGHT_MOTOR_LPWM, 0);
    analogWrite(RIGHT_MOTOR_RPWM, 200);
}
void reverseAndTurn()
{
    // Reverse
    digitalWrite(LEFT_MOTOR_L_EN, LOW);
    digitalWrite(LEFT_MOTOR_R_EN, HIGH);
    analogWrite(LEFT_MOTOR_LPWM, 200);
    analogWrite(LEFT_MOTOR_RPWM, 0);

    digitalWrite(RIGHT_MOTOR_L_EN, LOW);
    digitalWrite(RIGHT_MOTOR_R_EN, HIGH);
    analogWrite(RIGHT_MOTOR_LPWM, 200);
    analogWrite(RIGHT_MOTOR_RPWM, 0);

    delay(500); // Reverse for 3 second

    // Turn (e.g., 90 degrees to the right)
    digitalWrite(LEFT_MOTOR_L_EN, HIGH);
    digitalWrite(LEFT_MOTOR_R_EN, LOW);
    analogWrite(LEFT_MOTOR_LPWM, 0);
    analogWrite(LEFT_MOTOR_RPWM, 200);

    digitalWrite(RIGHT_MOTOR_L_EN, LOW);
    digitalWrite(RIGHT_MOTOR_R_EN, HIGH);
    analogWrite(RIGHT_MOTOR_LPWM, 200);
    analogWrite(RIGHT_MOTOR_RPWM, 0);

    delay(300); // Turn for 0.5 seconds (adjust as needed)

    stopAllMotors();
    delay(1000);
}

void operateMotors()
{
    if (leftMotorOn && rightMotorOn)
    {
        moveForward();
    }
    else
    {
        stopAllMotors();
    }

    if (brushMotorOn)
    {
        if (!operateBrushMotor())
        {
            reportMotorError("Brush");
        }
    }
    else
    {
        stopBrushMotor();
    }
    if (vacuumMotorOn)
    {
        if (!operateVacuumMotor())
        {
            reportMotorError("Vacuum");
        }
    }
    else
    {
        stopVacuumMotor();
    }
}

bool operateLeftMotor()
{
    digitalWrite(LEFT_MOTOR_L_EN, HIGH);
    digitalWrite(LEFT_MOTOR_R_EN,HIGH);
    analogWrite(LEFT_MOTOR_LPWM, 350);
    analogWrite(LEFT_MOTOR_RPWM, 0);
    return (analogRead(LEFT_MOTOR_LPWM) >= 10);
}

bool operateRightMotor()
{
    digitalWrite(RIGHT_MOTOR_L_EN, HIGH);
    digitalWrite(RIGHT_MOTOR_R_EN, HIGH);
    analogWrite(RIGHT_MOTOR_LPWM, 350);
    analogWrite(RIGHT_MOTOR_RPWM, 0);
    return (analogRead(RIGHT_MOTOR_LPWM) >= 10);
}

bool operateBrushMotor()
{
    digitalWrite(BRUSH_IN1, HIGH);
    digitalWrite(BRUSH_IN2, LOW);
    digitalWrite(BRUSH_IN3, HIGH);
    digitalWrite(BRUSH_IN4, LOW);
    // analogWrite(BRUSH_ENA, 255);
    // analogWrite(BRUSH_ENB, 255);
    // return (analogRead(BRUSH_ENA) >= 10 && analogRead(BRUSH_ENB) >= 10);
}

bool operateVacuumMotor()
{
    analogWrite(ESC_PIN, 128);
    return (analogRead(ESC_PIN) >= 10);
}

void stopLeftMotor()
{
    digitalWrite(LEFT_MOTOR_L_EN, LOW);
    digitalWrite(LEFT_MOTOR_R_EN, LOW);
    analogWrite(LEFT_MOTOR_LPWM, 0);
    analogWrite(LEFT_MOTOR_RPWM, 0);
}

void stopRightMotor()
{
    digitalWrite(RIGHT_MOTOR_L_EN, LOW);
    digitalWrite(RIGHT_MOTOR_R_EN, LOW);
    analogWrite(RIGHT_MOTOR_LPWM, 0);
    analogWrite(RIGHT_MOTOR_RPWM, 0);
}

void stopBrushMotor()
{
    // analogWrite(BRUSH_ENA, 0);
    // analogWrite(BRUSH_ENB, 0);
}

void stopVacuumMotor()
{
    analogWrite(ESC_PIN, 0);
}

void reportSensorError(const char *sensor)
{
    sensorError = true;
    if (millis() - lastErrorTime > ERROR_DISPLAY_INTERVAL)
    {
        Serial.print("Error: ");
        Serial.print(sensor);
        Serial.println(" sensor not working. Check connections.");
        lastErrorTime = millis();
    }
}

void reportMotorError(const char *motor)
{
    motorError = true;
    if (millis() - lastErrorTime > ERROR_DISPLAY_INTERVAL)
    {
        Serial.print("Error: ");
        Serial.print(motor);
        Serial.println(" motor not working. Check connections.");
        lastErrorTime = millis();
    }
}

int calculateTurnTime()
{
    // Adjust this based on your robot's turning speed and desired turn angle
    return 500; // 500ms for a ~90 degree turn, adjust as needed
}

void handleIRCommand(long command)
{
    switch (command)
    {
    case IR_BUTTON_POWER:
        toggleRobotPower();
        break;
    case IR_BUTTON_1:
        toggleLeftAndRightMotors();
        break;
    case IR_BUTTON_2:
        toggleBrushMotor();
        break;
    case IR_BUTTON_3:
        toggleVacuumMotor();
        break;
    case IR_BUTTON_4:
        toggleFrontSensor();
        break;
    case IR_BUTTON_5:
        toggleBottomSensor();
        break;
    case IR_BUTTON_6:
        checkSystemHealth();
        break;
    case IR_BUTTON_7:
        toggleServoMotor();
        break;
    default:
        Serial.println("Unknown IR command received");
    }
}

void toggleRobotPower()
{
    isRobotOn = !isRobotOn;
    Serial.println(isRobotOn ? "Robot turned ON" : "Robot turned OFF");
    if (isRobotOn)
    {
        leftMotorOn = true;
        rightMotorOn = true;
        brushMotorOn = true;
        vacuumMotorOn = true;
        frontSensorOn = true;
        bottomSensorOn = true;
        servoMotorOn = true;
        moveForward();
    }
    else
    {
        stopAllMotors();
    }
}

void toggleLeftAndRightMotors()
{
    leftMotorOn = !leftMotorOn;
    rightMotorOn = !rightMotorOn;
    Serial.println(leftMotorOn ? "Left and right motors activated" : "Left and right motors deactivated");
    if (leftMotorOn && rightMotorOn)
    {
        moveForward();
    }
    else
    {
        stopAllMotors();
    }
}

void toggleBrushMotor()
{
    brushMotorOn = !brushMotorOn;
    Serial.println(brushMotorOn ? "Brush motor activated" : "Brush motor deactivated");
}

void toggleVacuumMotor()
{
    vacuumMotorOn = !vacuumMotorOn;
    Serial.println(vacuumMotorOn ? "Vacuum motor activated" : "Vacuum motor deactivated");
}

void toggleFrontSensor()
{
    frontSensorOn = !frontSensorOn;
    Serial.println(frontSensorOn ? "Front sensor activated" : "Front sensor deactivated");
}

void toggleBottomSensor()
{
    bottomSensorOn = !bottomSensorOn;
    Serial.println(bottomSensorOn ? "Bottom sensor activated" : "Bottom sensor deactivated");
}

void toggleServoMotor()
{
    servoMotorOn = !servoMotorOn;
    Serial.println(servoMotorOn ? "Servo motor activated" : "Servo motor deactivated");
}

void stopAllMotors()
{
    stopLeftMotor();
    stopRightMotor();
    stopBrushMotor();
    stopVacuumMotor();
    Serial.println("All motors stopped.");
}

void moveForward()
{
    digitalWrite(LEFT_MOTOR_L_EN, HIGH);
    digitalWrite(LEFT_MOTOR_R_EN, HIGH);
    analogWrite(LEFT_MOTOR_LPWM, 0);
    analogWrite(LEFT_MOTOR_RPWM, 255);

    digitalWrite(RIGHT_MOTOR_L_EN, HIGH);
    digitalWrite(RIGHT_MOTOR_R_EN, HIGH);
    analogWrite(RIGHT_MOTOR_LPWM, 0 );
    analogWrite(RIGHT_MOTOR_RPWM, 255);
}

void blinkLED()
{
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
}

int measureDistance(int triggerPin, int echoPin) {
    if ((triggerPin == TRIGGER_PIN_FRONT && !frontSensorOn) ||
        (triggerPin == TRIGGER_PIN_BOTTOM && !bottomSensorOn)) {
        return -1; // Return error code for disabled sensor
    }

    const int NUM_READINGS = 3;
    int readings[NUM_READINGS];
    int validReadings = 0;

    for (int i = 0; i < NUM_READINGS; i++) {
        digitalWrite(triggerPin, LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPin, LOW);

        long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
        if (duration == 0) {
            readings[i] = -1; // Error: No echo received
        } else {
            readings[i] = duration * 0.034 / 2;
            validReadings++;
        }
        delay(10); // Short delay between readings
    }

    if (validReadings == 0) {
        return -1; // All readings failed
    }

    // Calculate average of valid readings
    int sum = 0;
    for (int i = 0; i < NUM_READINGS; i++) {
        if (readings[i] != -1) {
            sum += readings[i];
        }
    }
    return sum / validReadings;
}

void handleStepOrDrop()
{
    Serial.println("Step or drop detected. Handling...");
    stopAllMotors();
    delay(500);

    // Reverse a bit
    digitalWrite(LEFT_MOTOR_L_EN, LOW);
    digitalWrite(LEFT_MOTOR_R_EN, HIGH);
    analogWrite(LEFT_MOTOR_LPWM, 0);
    analogWrite(LEFT_MOTOR_RPWM, 255);

    digitalWrite(RIGHT_MOTOR_L_EN, LOW);
    digitalWrite(RIGHT_MOTOR_R_EN, HIGH);
    analogWrite(RIGHT_MOTOR_LPWM, 0);
    analogWrite(RIGHT_MOTOR_RPWM, 255);

    delay(500);

    stopAllMotors();
    delay(500);

    // Turn away from the edge
    if (movingRight)
    {
        // Turn left
        digitalWrite(LEFT_MOTOR_L_EN, LOW);
        digitalWrite(LEFT_MOTOR_R_EN, HIGH);
        analogWrite(LEFT_MOTOR_LPWM, 0);
        analogWrite(LEFT_MOTOR_RPWM, 255);

        digitalWrite(RIGHT_MOTOR_L_EN, HIGH);
        digitalWrite(RIGHT_MOTOR_R_EN, LOW);
        analogWrite(RIGHT_MOTOR_LPWM, 255);
        analogWrite(RIGHT_MOTOR_RPWM, 0);
    }
    else
    {
        // Turn right
        digitalWrite(LEFT_MOTOR_L_EN, HIGH);
        digitalWrite(LEFT_MOTOR_R_EN, LOW);
        analogWrite(LEFT_MOTOR_LPWM, 255);
        analogWrite(LEFT_MOTOR_RPWM, 0);

        digitalWrite(RIGHT_MOTOR_L_EN, LOW);
        digitalWrite(RIGHT_MOTOR_R_EN, HIGH);
        analogWrite(RIGHT_MOTOR_LPWM, 0);
        analogWrite(RIGHT_MOTOR_RPWM, 255);
    }

    delay(500);

    stopAllMotors();
    delay(500);

    moveForward();
    Serial.println("Step/drop handling complete. Resuming forward movement.");
}

// void updatePosition()
// {
//     switch (direction)
//     {
//     case 0:
//         currentY += GRID_SIZE;
//         break;
//     case 1:
//         currentX += GRID_SIZE;
//         break;
//     case 2:
//         currentY -= GRID_SIZE;
//         break;
//     case 3:
//         currentX -= GRID_SIZE;
//         break;
//     }
//     Serial.print("Updated position: X=");
//     Serial.print(currentX);
//     Serial.print(", Y=");
//     Serial.println(currentY);
// }

// void markCurrentPositionCleaned()
// {
//     int cellIndex = (currentY / GRID_SIZE) * MAX_GRID_X + (currentX / GRID_SIZE);
//     coverageGrid[cellIndex / 8] |= (1 << (cellIndex % 8));
//     Serial.println("Current position marked as cleaned.");
// }

void adjustZigzagWidth()
{
    zigzagWidth += (movingRight ? ZIGZAG_WIDTH_STEP : -ZIGZAG_WIDTH_STEP);
    zigzagWidth = constrain(zigzagWidth, MIN_ZIGZAG_WIDTH, MAX_ZIGZAG_WIDTH);
    Serial.print("Adjusted zigzag width: ");
    Serial.println(zigzagWidth);
}

bool isGridComplete()
{
    for (int i = 0; i < TOTAL_CELLS / 8 + 1; i++)
    {
        if (coverageGrid[i] != 0xFF)
        {
            return false;
        }
    }
    return true;
}

void resetGrid()
{
    memset(coverageGrid, 0, sizeof(coverageGrid));
    Serial.println("Grid reset complete.");
}

unsigned long calculateMoveInterval(int frontDistance)
{
    unsigned long interval = map(frontDistance, 10, 100, 50, 200);
    Serial.print("Calculated move interval: ");
    Serial.println(interval);
    return interval;
}

void printRobotStatus()
{
    Serial.println("\n--- Robot Status ---");
    Serial.println(isRobotOn ? "Robot: ON" : "Robot: OFF");
    Serial.println(leftMotorOn ? "Left motor: ON" : "Left motor: OFF");
    Serial.println(rightMotorOn ? "Right motor: ON" : "Right motor: OFF");
    Serial.println(brushMotorOn ? "Brush motor: ON" : "Brush motor: OFF");
    Serial.println(vacuumMotorOn ? "Vacuum motor: ON" : "Vacuum motor: OFF");
    Serial.println(frontSensorOn ? "Front sensor: ON" : "Front sensor: OFF");
    Serial.println(bottomSensorOn ? "Bottom sensor: ON" : "Bottom sensor: OFF");
    Serial.println(servoMotorOn ? "Servo motor: ON" : "Servo motor: OFF");
    Serial.println("--------------------\n");
}

void displayActiveErrors()
{
    if (millis() - lastErrorTime > ERROR_DISPLAY_INTERVAL)
    {
        if (sensorError)
        {
            Serial.println("Active Error: Ultrasonic sensor malfunction.");
        }
        if (motorError)
        {
            Serial.println("Active Error: Motor malfunction.");
        }
        lastErrorTime = millis();
    }
}

void checkSystemHealth()
{
    if (sensorError || motorError)
    {
        Serial.println("System health check: Issues detected. Please check error messages.");
    }
    else
    {
        Serial.println("System health check: All systems operational.");
    }
}