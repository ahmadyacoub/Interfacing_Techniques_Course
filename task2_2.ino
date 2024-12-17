#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal.h>

// Motor Control Pins
#define ENA 11   // PWM for motor speed
#define IN1 9    // Motor direction pin 1
#define IN2 10   // Motor direction pin 2

// Encoder Pins
#define ENCODER_A 2  // Encoder channel A (Interrupt pin)
#define ENCODER_B 3  // Encoder channel B (Interrupt pin)

// Encoder Variables
volatile long encoderCount = 0;  // Tracks the number of pulses
int lastEncoded = 0;             // Previous encoded state

// Motor Speed and MPU Control Variables
int motorSpeed = 0;              // Motor speed (0-255)
const int maxSpeed = 255;        // Max motor speed

// MPU6050 Object
MPU6050 mpu;

// LCD Pins and Initialization (using standard LiquidCrystal library)
#define RS 4   // Register Select pin
#define E 5    // Enable pin
#define D4 6   // Data Pin 4
#define D5 7   // Data Pin 5
#define D6 8   // Data Pin 6
#define D7 12  // Data Pin 7

LiquidCrystal lcd(RS, E, D4, D5, D6, D7);  // Create LCD object

// Motor State Variables
String motorState = "Stopped";  // Initial motor state

// Function Prototypes
void encoderISR();
void moveMotorForward(int speed);
void moveMotorBackward(int speed);
void stopMotor();

void setup() {
  // Initialize motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Attach interrupts for the encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  // Start Serial communication
  Serial.begin(9600);
  Serial.println("System initializing...");

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful.");
  } else {
    Serial.println("MPU6050 connection failed. Check wiring.");
    while (1);
  }

  // Initialize LCD
  lcd.begin(20, 4);  // Set LCD dimensions (20 columns, 4 rows)
  lcd.clear();       // Clear the LCD screen
  lcd.setCursor(0, 0);
  lcd.print("System initializing...");
  delay(2000);  // Display initialization message for 2 seconds
}

void loop() {
  // Get MPU6050 readings
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate tilt in degrees (pitch)
  float pitch = atan2(ay, az) * 180 / PI;

  // Map tilt to motor speed range
  motorSpeed = map(abs(pitch), 0, 90, 0, maxSpeed);

  // Control motor direction based on tilt
  if (pitch > 5) {  // Tilt forward
    moveMotorForward(motorSpeed);
    motorState = " CW";
  } else if (pitch < -5) {  // Tilt backward
    moveMotorBackward(motorSpeed);
    motorState = "CCW";
  } else {
    stopMotor();
    motorState = "Stopped";
  }

  // Update LCD with encoder cycle, pitch, and motor state
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Encoder cycle: ");
  lcd.print(encoderCount / 815);  // Adjust encoder count for cycles

  lcd.setCursor(0, 1);
  lcd.print("Pitch: ");
  lcd.print(pitch);

  lcd.setCursor(0, 2);
  lcd.print("Motor: ");
  lcd.print(motorState);  // Display motor status

  // Print encoder count, pitch, and motor state to Serial Monitor for debugging
  Serial.print("Encoder cycle: ");
  Serial.print(encoderCount / 815);  // Adjust encoder count for cycles
  Serial.print("\tPitch: ");
  Serial.print(pitch);
  Serial.print("\tMotor: ");
  Serial.println(motorState);

  delay(500);  // Small delay to update the LCD smoothly
}

// Function to move the motor forward
void moveMotorForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
}

// Function to move the motor backward
void moveMotorBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

// Function to stop the motor
void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

// Interrupt Service Routine for the encoder
void encoderISR() {
  int MSB = digitalRead(ENCODER_A);  // Read encoder channel A
  int LSB = digitalRead(ENCODER_B);  // Read encoder channel B

  int encoded = (MSB << 1) | LSB;    // Combine both signals
  int sum = (lastEncoded << 2) | encoded;  // State transition

  // Determine direction and adjust encoder count
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;
  }

  lastEncoded = encoded;  // Update the last encoded state
}
