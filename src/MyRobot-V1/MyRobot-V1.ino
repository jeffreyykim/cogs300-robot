// MyRobot-V1.ino
// This file must be named the same as your sketch folder

// Motor A pins (left motor)
int enA = 9;   // Enable pin for Motor A — must be a PWM-capable pin
int in1 = 8;   // Direction control pin 1 for Motor A
int in2 = 7;   // Direction control pin 2 for Motor A

// Motor B pins (right motor)
int enB = 3;   // Enable pin for Motor B — must be a PWM-capable pin
int in3 = 6;   // Direction control pin 1 for Motor B
int in4 = 5;   // Direction control pin 2 for Motor B

void setup() {
    // Initialize serial communication for logging
    Serial.begin(9600);
    
    // Set motor control pins as outputs
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void loop() {
    // put your main code here, to run repeatedly:

    // Drive for 1 second
    logInfo(Serial, "Driving");
    drive(in1, in2, enA);
    delay(1000);

    // Stop for 1 second
    logInfo(Serial, "Stopping");
    stop(in1, in2, enA);
    delay(1000);

    // TODO: add your own driving functions here
}
