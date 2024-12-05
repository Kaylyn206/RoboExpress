#include <Servo.h>

// Motor Driver Pin Definitions
#define ENA 5  // PWM for Motor A
#define IN1 4  // Motor A forward
#define IN2 2  // Motor A backward
#define ENB 6  // PWM for Motor B
#define IN3 8  // Motor B forward
#define IN4 7  // Motor B backward

// IR sensor pin definitions
const int ir1Pin = 12;  // IR Sensor 1 connected to pin 12
const int ir2Pin = 13;  // IR Sensor 2 connected to pin 13

// Servo Motors
Servo servo1, servo2, servo3, servo4;

int servo3Position = 90; // Initial position of servo3 (90°)

int motorSpeed = 100;  // Speed value (0 to 255)
char currentMode = 'K';  // Default mode (Manual)

void setup() {
  // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // IR sensor pins setup
  pinMode(ir1Pin, INPUT);
  pinMode(ir2Pin, INPUT);

  // Servo motors setup
  servo1.attach(3);   // 360-degree servo
  servo2.attach(9);   // 360-degree servo
  servo3.attach(10);  // 180-degree servo
  servo4.attach(11);  // 360-degree servo

  Serial.begin(9600); // Set baud rate for Bluetooth communication
}

void loop() {
  if (Serial.available() > 0) {
    char receivedCommand = Serial.read();

    switch (receivedCommand) {
      case 'K': switchMode('K'); break; // Manual mode
      case 'C': switchMode('C'); break; // Auto mode
      case 'P': switchMode('P'); break; // Servo control set 1
      case 'N': switchMode('N'); break; // Servo control set 2
      default:
        handleCommand(receivedCommand);
        break;
    }
  }

  if (currentMode == 'C') {
    autoControl();
  }
}

// Switch between modes
void switchMode(char mode) {
  currentMode = mode;
  stopAllActions();
  Serial.print("Switched to mode: ");
  Serial.println(currentMode);
}

// Handle commands based on the current mode
void handleCommand(char command) {
  if (currentMode == 'K') {
    driveMotors(command);  // Manual driving
  } else if (currentMode == 'P') {
    controlServos(command, servo1, servo2); // Control servo1 and servo2
  } else if (currentMode == 'N') {
    controlServos(command, servo3, servo4); // Control servo3 and servo4
  }
}

// Manual driving commands
void driveMotors(char command) {
  switch (command) {
    case 'F': setMotorState(HIGH, LOW, HIGH, LOW); break;  // Forward
    case 'B': setMotorState(LOW, HIGH, LOW, HIGH); break;  // Backward
    case 'L': setMotorState(LOW, HIGH, HIGH, LOW); break;  // Left
    case 'R': setMotorState(HIGH, LOW, LOW, HIGH); break;  // Right
    default: stopMotors(); break;
  }
}

// Autonomous driving based on IR sensors
void autoControl() {
  int ir1State = digitalRead(ir1Pin);
  int ir2State = digitalRead(ir2Pin);

  if (ir1State == LOW && ir2State == LOW) {
    setMotorState(HIGH, LOW, HIGH, LOW); // Forward
  } else if (ir1State == HIGH && ir2State == LOW) {
    setMotorState(HIGH, LOW, LOW, HIGH); // Turn Left
  } else if (ir1State == LOW && ir2State == HIGH) {
    setMotorState(LOW, HIGH, HIGH, LOW); // Turn Right
  } else {
    stopMotors(); // Stop
  }
}

// Control servos dynamically based on the mode
void controlServos(char command, Servo &servoA, Servo &servoB) {
  // Determine which servos are being controlled
  if (&servoA == &servo1 || &servoA == &servo2) { // Servo1 and Servo2 (Mode 'P')
    switch (command) {
      case 'F': servoA.write(180); break; // Move forward
      case 'B': servoA.write(0); break;   // Move backward
      case 'L': servoB.write(0); break;   // Turn left
      case 'R': servoB.write(180); break; // Turn right
      default:
        servoA.write(90); // Neutral position
        servoB.write(90); // Neutral position
        break;
    }
  } else if (&servoA == &servo3 || &servoA == &servo4) { // Servo3 and Servo4 (Mode 'N')
    if (&servoA == &servo3) { // Special handling for servo3
      switch (command) {
        case 'F': 
          servo3Position += 5; // Increment position gradually
          if (servo3Position > 180) servo3Position = 180;
          servoA.write(servo3Position);
          break;
        case 'B': 
          servo3Position -= 5; // Decrement position gradually
          if (servo3Position < 0) servo3Position = 0;
          servoA.write(servo3Position);
          break;
        case 'L': servoB.write(0); break;   // Move left
        case 'R': servoB.write(180); break; // Move right
        default:
          // Do nothing; keep servo3 at the last position
          break;
      }
    }
  }
}


// Helper function to set motor states
void setMotorState(int in1, int in2, int in3, int in4) {
  digitalWrite(IN1, in1);
  digitalWrite(IN2, in2);
  digitalWrite(IN3, in3);
  digitalWrite(IN4, in4);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

// Stop all motors
void stopMotors() {
  setMotorState(LOW, LOW, LOW, LOW);
}

// Stop the specific servos
void stopServos(Servo &servoA, Servo &servoB) {
  servoA.write(90); // Stop servo A
  servoB.write(90); // Stop servo B
}

// Stop all actions (motors and servos)
void stopAllActions() {
  stopMotors();
  stopServos(servo1, servo2);
  stopServos(servo3,servo4);
}