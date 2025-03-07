#define MOTOR1_IN1 5    // Motor 1 - High Torque
#define MOTOR1_IN2 6    
#define MOTOR2_IN1 9    // Motor 2 - Low Torque
#define MOTOR2_IN2 10    

#define IR_SENSOR 2     // IR Sensor pin
int objectCount = 0;

void setup() {
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);
    pinMode(IR_SENSOR, INPUT);

    Serial.begin(9600);  // Start serial communication
}

void loop() {
    // Start both motors to move the conveyor
    moveMotor(MOTOR1_IN1, MOTOR1_IN2); // High torque motor
    moveMotor(MOTOR2_IN1, MOTOR2_IN2); // Low torque motor

    // Check IR sensor for object detection
    if (digitalRead(IR_SENSOR) == LOW) {
        objectCount++; // Increment count when object passes
        Serial.print("Object Count: ");
        Serial.println(objectCount);
        delay(500); // Debounce delay
    }
}

// Function to move motor forward
void moveMotor(int in1, int in2) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}
