// ultarsonic pins
const int trigPin = 9;
const int echoPin = 8;
//  LEDs pins
const int greenLED = 2;
const int yellowLED = 3;
const int redLED = 4;
// pot pins
const int potPin = A0;

// distance thresholds as a start
int greenThreshold = 20;
int yellowThreshold = 10;

void setup() { // pins setup & for serial monitor
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(greenLED, OUTPUT);
    pinMode(yellowLED, OUTPUT);
    pinMode(redLED, OUTPUT);
    Serial.begin(9600);
}

long measureDistance() { // a read ultrasonic distance  
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2;
    return distance;
}

void updateThresholds() {
    int potValue = analogRead(potPin); 
  // Read potentiometer value (0 to 1023) as a deafult for arduino
  //uno as it takes 10 bits from input
    
// changing the potentiometer values changes the range of thresholds
  greenThreshold = map(potValue, 0, 1023, 15, 30);
    yellowThreshold = greenThreshold / 2;
}

void controlLEDs(long distance) { 
  // controls the leds for every change and print on the monitor to debug
    if (distance > greenThreshold) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(yellowLED, LOW);
        digitalWrite(redLED, LOW);
        Serial.println("Green LED ON");
    } else if (distance > yellowThreshold) {
        digitalWrite(greenLED, LOW);
        digitalWrite(yellowLED, HIGH);
        digitalWrite(redLED, LOW);
        Serial.println("Yellow LED ON");
    } else {
        digitalWrite(greenLED, LOW);
        digitalWrite(yellowLED, LOW);
        digitalWrite(redLED, HIGH);
        Serial.println("Red LED ON");
    }
}

void loop() {
    long distance = measureDistance();
    updateThresholds();
    controlLEDs(distance);

    // Print values to Serial Monitor for debugging
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Green Threshold: ");
    Serial.print(greenThreshold);
    Serial.print(" cm, Yellow Threshold: ");
    Serial.println(yellowThreshold);
    
    delay(100);
}