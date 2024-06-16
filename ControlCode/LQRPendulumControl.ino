#include <AccelStepper.h>
#include <SPI.h>
#include <TimerThree.h>

// Pin Definitions
#define CS 40 // Chip or Slave select
#define STEP_PIN 37
#define DIR_PIN 39
#define LS 18
#define RS 19
#define ENA 34

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

// LQR control variables
double setpoint = 0.0;  // Desired angular position (set by user)
volatile double input = 0.0; // Current angular position
double output = 0.0;    // Output from LQR controller
double outputLimit = 4000;

double startUpSpeed = 500;

// LQR gain matrix (replace with your calculated values)
double K[2] = {1159.071503149265, 29.2210085894763};

//double K[4] = {-31.6228,-62.4061,451.2577,104.6918};

// AccelStepper instance
double maxSpeed = 5200;
double acceleration = 4000;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
double ratio = maxSpeed / outputLimit;

// State vector components
double cartPosition = 0.0;
double cartVelocity = 0.0;
double pendulumAngle = 0.0;
double pendulumAngularVelocity = 0.0;
double prevPendulumAngle = 0.0;
unsigned long lastTime = 0;

double refPosition = 0.0;

void setup() {
    // Initialize pins and SPI
    pinMode(CS, OUTPUT); // Slave Select
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
    stepper.setMaxSpeed(maxSpeed); // Adjust as necessary
    stepper.setAcceleration(acceleration);

    // Initialize Timer3 for stepper speed control
    Timer3.initialize(2000000 / stepper.speed()); // Interval in microseconds for the step speed
    Timer3.attachInterrupt(stepperInterrupt);

    refPosition = stepper.currentPosition();
    Serial.println(refPosition);

    // Uncomment these lines if needed
    // setZeroPoint();
    // findCenterPosition();
    // delay(100);
    // initialSwing();
    // delay(100);
}

void loop() {
    while (!stabilize) {
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
            stepper.setSpeed(startUpSpeed); // Move right
        } else {
            stepper.setSpeed(-startUpSpeed); // Move left
        }

        if (abs(input) < 2.0) {
            break;
        }
    }
    stabilize = true;
    // Serial.println("Balance");

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

        // Read encoder and update LQR
        readEncoder();
        updateState();
        computeLQR();
        updateSpeed(output);
        Serial.println(output);
    }
}

// LQR control function
void computeLQR() {
    // Define the state vector [angle, angular velocity]
    double state[2] = {pendulumAngle, pendulumAngularVelocity};

    // Compute the control input u = -K * state
    output = -(K[0] * state[0] + K[1] * state[1]);
    output = constrain(output, -outputLimit, outputLimit); // Limit the output to the set range
}

// Uncomment this function if using a full state vector for LQR control
// void computeLQR() {
//   // Define the state vector [cart position, cart velocity, angle, angular velocity]
//   double state[4] = {cartPosition - refPosition, cartVelocity, pendulumAngle, pendulumAngularVelocity};

//   // Compute the control input u = -K * state
//   output = -(K[0] * state[0] + K[1] * state[1] + K[2] * state[2] + K[3] * state[3]);
//   output = constrain(output, -outputLimit, outputLimit); // Limit the output to the set range
// }

// Speed control function
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

// State update function
void updateState() {
    unsigned long currentTime = millis();
    double dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    pendulumAngle = input;
    pendulumAngularVelocity = (pendulumAngle - prevPendulumAngle) / dt;
    prevPendulumAngle = pendulumAngle;
}

// Uncomment this function if using a full state vector for state update
// void updateState() {
//   unsigned long currentTime = millis();
//   double dt = (currentTime - lastTime) / 1000.0;
//   lastTime = currentTime;

//   cartPosition = stepper.currentPosition();
//   cartVelocity = stepper.speed();
//   pendulumAngle = input;
//   pendulumAngularVelocity = (pendulumAngle - prevPendulumAngle) / dt;
//   prevPendulumAngle = pendulumAngle;
// }

// Timer interrupt for stepper control
void stepperInterrupt() {
    stepper.runSpeed(); // Perform a single step
}

// SPI transaction function
uint8_t SPI_T(uint8_t msg) {
    uint8_t msg_temp = 0; // Variable to hold received data
    digitalWrite(CS, LOW); // Select SPI device
    msg_temp = SPI.transfer(msg); // Send and receive
    digitalWrite(CS, HIGH); // Deselect SPI device
    return (msg_temp); // Return received byte
}

// Encoder reading function
void readEncoder() {
    uint8_t received = 0xA5; // Just a temp variable
    ABSposition = 0; // Reset position variable

    digitalWrite(CS, LOW);
    SPI_T(0x10); // Issue read command
    received = SPI_T(0x00); // Issue NOP to check if encoder is ready to send

    while (received != 0x10) { // Loop while encoder is not ready to send
        received = SPI_T(0x00); // Check again if encoder is still working
        delay(2);
    }

    temp[0] = SPI_T(0x00); // Receive MSB
    temp[1] = SPI_T(0x00); // Receive LSB
    digitalWrite(CS, HIGH);
    temp[0] &= ~0xF0; // Mask out the first 4 bits

    ABSposition = temp[0] << 8; // Shift MSB to correct ABSposition in ABSposition message
    ABSposition += temp[1]; // Add LSB to ABSposition message to complete message

    if (ABSposition != ABSposition_last) { // If nothing has changed don't waste time sending position
        ABSposition_last = ABSposition; // Set last position to current position
        input = (ABSposition * 0.08789) - 180; // Set input for LQR controller
    }
    if (abs(input) < 0.2) {
        input = 0.0;
    }
}

// Function to set zero point
void setZeroPoint() {
    digitalWrite(CS, LOW); // Select SPI device
    SPI.transfer(0x70); // Send zeroing command (replace 0x70 with the correct command from the datasheet)
    digitalWrite(CS, HIGH); // Deselect SPI device
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
    stepper.setSpeed(-1000); // Set speed to negative for reverse direction
    unsigned long startTime = millis();
    const unsigned long reverseDuration = 300; // Run in reverse for 300 milliseconds

    while (millis() - startTime < reverseDuration) {
        stepper.runSpeed();
    }

    digitalWrite(ENA, HIGH); // Stop the motor
}

void rightStop() {
    stepper.setSpeed(1000); // Set speed to positive for forward direction
    unsigned long startTime = millis();
    const unsigned long reverseDuration = 300; // Run in reverse for 300 milliseconds

    while (millis() - startTime < reverseDuration) {
        stepper.runSpeed();
    }

    digitalWrite(ENA, HIGH); // Stop the motor
}

// Function to find center position
void findCenterPosition() {
    // Move to the left until the left switch is triggered
    stepper.setSpeed(400); // Move left slowly
    while (!leftStopTriggered) {
        stepper.runSpeed();
    }
    leftStopPosition = stepper.currentPosition();
    Serial.print(leftStopPosition);
    leftStopTriggered = false;

    // Move to the right until the right switch is triggered
    stepper.setSpeed(-400); // Move right slowly
    while (!rightStopTriggered) {
        stepper.runSpeed();
    }
    rightStopPosition = stepper.currentPosition();
    Serial.print(rightStopTriggered);
    rightStopTriggered = false;

    // Calculate the center position
    long centerPosition = (leftStopPosition + rightStopPosition) / 2;
    refPosition = centerPosition;
    Serial.print(centerPosition);

    // Move to the center position quickly
    stepper.moveTo(centerPosition);
    stepper.setSpeed(400);
    while (stepper.distanceToGo() != 0) {
        stepper.runSpeedToPosition();
    }
}
