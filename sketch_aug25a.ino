// Motor and control pin definitions
int speed_A = 1; // enA
int in_A1 = 2;   // in1
int in_A2 = 3;   // in2
int in_B1 = 4;   // in3
int in_B2 = 5;   // in4
int speed_B = 6; // enB

// Ultrasonic sensor pins
const int trigLeft = 7;  // Trigger pin for left ultrasonic sensor
const int echoLeft = 8;  // Echo pin for left ultrasonic sensor
const int trigRight = 9; // Trigger pin for right ultrasonic sensor
const int echoRight = 10; // Echo pin for right ultrasonic sensor

// IR sensor pin
const int irBottom = 11;   // Bottom IR sensor pin

// Obstacle detection threshold (in cm)
const int distanceThreshold = 25;

void setup() {
  // Set motor pins as outputs
  pinMode(speed_A, OUTPUT);
  pinMode(in_A1, OUTPUT);
  pinMode(in_A2, OUTPUT);
  pinMode(speed_B, OUTPUT);
  pinMode(in_B1, OUTPUT);
  pinMode(in_B2, OUTPUT);

  // Initialize motor speed to low
  digitalWrite(speed_A, LOW);
  digitalWrite(speed_B, LOW);

  // Initialize serial communication
  Serial.begin(9600);

  // Set ultrasonic sensor pins
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  // Set IR sensor pin as input
  pinMode(irBottom, INPUT);
}

void loop() {
  // Measure distances using ultrasonic sensors
  float distanceLeft = getDistance(trigLeft, echoLeft);
  float distanceRight = getDistance(trigRight, echoRight);

  Serial.print("Distance Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm, Distance Right: ");
  Serial.print(distanceRight);
  Serial.println(" cm");

  // Check for black line detection using bottom IR sensor
  if (detectBlackLine()) {
    Serial.println("Black line detected! Stopping...");
    stopMotors();
  } else {
    // Obstacle avoidance logic
    if (distanceLeft < distanceThreshold || distanceRight < distanceThreshold) {
      Serial.println("Obstacle detected! Stopping...");
      stopMotors(); // Stop motors if an obstacle is too close
      delay(100);  // Pause to avoid collision
      reverseMotors(); // Reverse the robot to move away from the obstacle
      delay(100);  // Move backward for a short time
    } else {
      Serial.println("No obstacle detected. Moving forward...");
      moveForward(); // Move forward if no obstacle is detected
    }
  }

  delay(100); // Short delay for stability
}

// Function to calculate distance using an ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2; // Convert to cm
  return distance;
}

// Function to detect black line using bottom IR sensor
bool detectBlackLine() {
  int irValue = digitalRead(irBottom);

  // Assuming the sensor reads HIGH for black and LOW for white/ground
  if (irValue == HIGH) {
    return true;  // Black line detected
  }
  return false;  // No black line detected
}

// Function to stop motors
void stopMotors() {
  analogWrite(speed_A, 0);
  analogWrite(speed_B, 0);
  digitalWrite(in_A1, LOW);
  digitalWrite(in_A2, LOW);
  digitalWrite(in_B1, LOW);
  digitalWrite(in_B2, LOW);
}

// Function to reverse motors
void reverseMotors() {
  digitalWrite(in_A1, HIGH);
  digitalWrite(in_A2, LOW);
  digitalWrite(in_B1, HIGH);
  digitalWrite(in_B2, LOW);
  analogWrite(speed_A, 125);
  analogWrite(speed_B, 125);
}

// Function to move forward
void moveForward() {
  digitalWrite(in_A1, LOW);
  digitalWrite(in_A2, HIGH);
  digitalWrite(in_B1, LOW);
  digitalWrite(in_B2, HIGH);
  analogWrite(speed_A, 125);
  analogWrite(speed_B,125);
}