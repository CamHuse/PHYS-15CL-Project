#include <AccelStepper.h>
#include <PID_v1.h>
#include <SPI.h>
#include <TimerThree.h>
#include <RTClib.h>

// Pin Definitions
#define CS 40 // Chip or Slave select
#define STEP_PIN 37
#define DIR_PIN 39
#define LS 18
#define RS 19
#define ENA 34

// Real-time clock instance
RTC_DS3231 rtc;

// Flags and control variables
volatile bool rightStopTriggered = false;
volatile bool leftStopTriggered = false;
volatile bool stabilize = false;

double last_output = 0.0;
long leftStopPosition = 0;
long rightStopPosition = 0;

// Encoder variables
uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t temp[2]; 

// PID control variables
double setpoint = 0.0;
volatile double input = 0.0;
double output = 0.0;
double outputLimit = 100;

// PID tuning parameters
float Kp = 1000.0;
float Ki = 0.0;
float Kd = 1.5;

// AccelStepper instance
double maxSpeed = 20000;
double acceleration = 7000;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// PID instance
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
double ratio = maxSpeed / outputLimit;

// Setup function
void setup() {
    // Initialize pins and SPI
    pinMode(CS, OUTPUT); 
    digitalWrite(CS, HIGH);
    SPI.begin();
    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    SPI.setClockDivider(SPI_CLOCK_DIV32);
    
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("starting");
    Serial.flush();
    delay(2000);

    // Initialize control pins and interrupts
    pinMode(ENA, OUTPUT);
    pinMode(LS, OUTPUT);
    pinMode(RS, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(LS), leftStopISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RS), rightStopISR, CHANGE);

    // Initialize stepper motor
    digitalWrite(ENA, LOW);
    stepper.setMaxSpeed(maxSpeed); 
    stepper.setAcceleration(acceleration);

    // Initialize PID controller
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-outputLimit, outputLimit); 

    // Initialize Timer3 for stepper speed control
    Timer3.initialize(2000000 / stepper.speed());
    Timer3.attachInterrupt(stepperInterrupt);
}

// Main loop
void loop() {

    while (true) {
        // Handle end stops
        if (rightStopTriggered) {
            rightStop();
            rightStopTriggered = false;
        }
        if (leftStopTriggered) {
            leftStop();
            leftStopTriggered = false;
        }

        // Read encoder and control movement
        readEncoder();
        Serial.println(input); 
        
        // Swing-up logic
        if (input > 0) {
            stepper.setSpeed(1100); // Move right
        } else {
            stepper.setSpeed(-1100); // Move left
        }

        if (abs(input) < 1.0) {
            break;
        }
    }
    stabilize = true;

    // Stabilization loop
    while (stabilize) {
        // Handle end stops
        if (rightStopTriggered) {
            rightStop();
            rightStopTriggered = false;
        }
        if (leftStopTriggered) {
            leftStop();
            leftStopTriggered = false;
        }

        // Read encoder and update PID
        readEncoder();
        last_output = output;
        myPID.Compute();
        updateSpeed(output * ratio);
        Serial.println(stepper.speed());
    }
}

// PID and speed control functions
void updateSpeed(double targetSpeed) {
    static double previousSpeed = last_output;
    double speedDifference = targetSpeed - previousSpeed;

    if (abs(speedDifference) < acceleration) {
        previousSpeed = targetSpeed;
    } else if (speedDifference > 0) {
        previousSpeed += acceleration;
    } else {
        previousSpeed -= acceleration;
    }

    stepper.setSpeed(previousSpeed);
}

// Initial swing function
void initialSwing() {
    unsigned long startTime = millis();
    const unsigned long swingDuration = 300; 

    // Move the cart right briefly
    while (millis() - startTime < swingDuration) {
        stepper.runSpeed();
    }

    // Move the cart left briefly
    startTime = millis();
    while (millis() - startTime < swingDuration / 2) {
        stepper.setSpeed(-1000);
    }
    while (millis() - startTime < swingDuration) {
        stepper.setSpeed(1000);
    }
}

// Timer interrupt for stepper control
void stepperInterrupt() {
    stepper.runSpeed(); 
}

// SPI transaction function
uint8_t SPI_T(uint8_t msg) {
    uint8_t msg_temp = 0; 
    digitalWrite(CS, LOW); 
    msg_temp = SPI.transfer(msg); 
    digitalWrite(CS, HIGH); 
    return (msg_temp); 
}

// Encoder reading function
void readEncoder() {
    uint8_t received = 0xA5; 
    ABSposition = 0; 

    digitalWrite(CS, LOW);
    SPI_T(0x10); 
    received = SPI_T(0x00); 

    while (received != 0x10) {
        received = SPI_T(0x00); 
        delay(2);
    }

    temp[0] = SPI_T(0x00); 
    temp[1] = SPI_T(0x00); 
    digitalWrite(CS, HIGH);
    temp[0] &= ~0xF0; 

    ABSposition = temp[0] << 8; 
    ABSposition += temp[1]; 

    if (ABSposition != ABSposition_last) {
        ABSposition_last = ABSposition; 
        input = round((ABSposition * 0.17579) - 360); 
    }
    if (abs(input) < 5.0) {
        input = 0.0;
    }
}

// Function to set zero point
void setZeroPoint() {
    digitalWrite(CS, LOW); 
    SPI.transfer(0x70); 
    digitalWrite(CS, HIGH); 
}

// Stop switch ISR functions
void leftStopISR() {
    leftStopTriggered = true;
}

void rightStopISR() {
    rightStopTriggered = true;
}

// Stop switch handling functions
void leftStop() {
    stepper.setSpeed(-1000);  
    unsigned long startTime = millis();
    const unsigned long reverseDuration = 300;  

    while (millis() - startTime < reverseDuration) {
        stepper.runSpeed();
    }

    digitalWrite(ENA, HIGH); 
}

void rightStop() {
    stepper.setSpeed(1000);  
    unsigned long startTime = millis();
    const unsigned long reverseDuration = 300;  

    while (millis() - startTime < reverseDuration) {
        stepper.runSpeed();
    }

    digitalWrite(ENA, HIGH); 
}

// Function to find center position
void findCenterPosition() {
    // Move to the left until the left switch is triggered
    stepper.setSpeed(400); 
    while (!leftStopTriggered) {
        stepper.runSpeed();
    }
    leftStopPosition = stepper.currentPosition();
    Serial.print(leftStopPosition);
    leftStopTriggered = false;

    // Move to the right until the right switch is triggered
    stepper.setSpeed(-400); 
    while (!rightStopTriggered) {
        stepper.runSpeed();
    }
    rightStopPosition = stepper.currentPosition();
    Serial.print(rightStopTriggered);
    rightStopTriggered = false;

    // Calculate the center position
    long centerPosition = (leftStopPosition + rightStopPosition) / 2;
    Serial.print(centerPosition);

    // Move to the center position quickly
    stepper.moveTo(centerPosition);
    stepper.setSpeed(400);
    while (stepper.distanceToGo() != 0) {
        stepper.runSpeedToPosition();
    }
}
