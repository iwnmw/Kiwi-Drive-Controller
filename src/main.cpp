#include <Arduino.h>
#include <Encoder.h>
#include "PIDController.h"
#include <vector>
#include <iostream>
#include <cmath>



// Define pin variables
const int INA = 2;
const int INB = 4;
const int PWM_PIN = 3;

// Define minimum, maximum speed and commanded speed (manually update these while testing)
const int minSpeed = -530; // Minimum speed in RPM
const int maxSpeed = 530; // Maximum speed in RPM
float RPMTarget = 330.0; // Commanded speed in RPM

// Define variables for encoder reading and initialize object
const int encoderPinA = 5; // Must be interrupt capable; keep in mind if you change to another board
const int encoderPinB = 6;
Encoder myEnc(encoderPinA, encoderPinB);

// Define values for converting the encoder readings to speed (RPM)
const int encoderCPR = 1200; // Counts per revolution of the encoder (output shaft)
unsigned long lastTime = 0;

// Initialize the PID controller object
// Note: PID controller controls rate of change of PWM signal, not speed directly, which is why the maximum and minimum outputs are set as they are
// First value is proportional gain, second is integral gain, third is derivative gain
PIDController PID(6.5, 4.0, 0.0, -255, 255); // PID gains and output limits



// Define a function to convert from rotational speed to PWM output
// Note: negative PWM values will not actually be output as negative PWM values, but will simply reverse the motor direction and use the magnitue for speed
int speedToPWM(int commandedSpeed) {
  // Assuming speed is in RPM and we want to convert it to a PWM value
  return map(commandedSpeed, minSpeed, maxSpeed, -255, 255);
}

// Define a function to send a motor output
void sendMotorOutput(int speed) {

  if (speed > 0) {
    digitalWrite(INA, HIGH);
    digitalWrite(INB, LOW);
  } else if (speed < 0) {
    digitalWrite(INA, LOW);
    digitalWrite(INB, HIGH);
  } else {
    digitalWrite(INA, LOW);
    digitalWrite(INB, LOW);
  }
  analogWrite(PWM_PIN, abs(speed));
}

// Define a function to do the kiwi drive inverse kinematics
float RAD_SEC_TO_RPM = 9.549296596425384; // Conversion factor from radians per second to RPM
std::vector<float> kiwiDriveInverseKinematics(float angularVelocity, float xVelocity, float yVelocity) {
    // Define the wheel radius and distance between wheels
    const float wheelCenterRadius = 12.0f; // Distance from center of rover to wheel, inches
    const float wheelRadius = 2.375f; // Radius of the wheel, inches

    // Form the H and Vb matrices for inverse kinematics
    std::vector<std::vector<float>> H = {
        {-wheelCenterRadius, 1, 0},
        {-wheelCenterRadius, -1/2, -static_cast<float>(sin(M_PI/3))},
        {-wheelCenterRadius, -0.5f, static_cast<float>(sin(M_PI/3))}
    };

    std::vector<float> Vb = {angularVelocity, xVelocity, yVelocity};

    std::vector<float> wheelSpeedsInchesPerSecond = multiplyMatrix(H, Vb); 
    std::vector<float> wheelSpeeds(wheelSpeedsInchesPerSecond.size());
    for (size_t i = 0; i < wheelSpeedsInchesPerSecond.size(); ++i) {
        wheelSpeeds[i] = (wheelSpeedsInchesPerSecond[i] / wheelRadius) * RAD_SEC_TO_RPM; // Convert from inches/sec to RPM
    }
    return wheelSpeeds;
}

// Define a function to do small matrix multiplication
std::vector<float> multiplyMatrix(const std::vector<std::vector<float>>& A, const std::vector<float>& B) {
    std::vector<float> result(A.size(), 0.0f);
    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < B.size(); ++j) {
            result[i] += A[i][j] * B[j];
        }
    }
    return result;
}


void setup() {
  Serial.begin(115200);

  // Initialize pins for motor control
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  Serial.println("Motor Control Initialized");
}

long oldPosition  = 0;



void loop() {
  // Read the encoder position and calculate speed
  long newPosition = myEnc.read();
  unsigned long currentTime = millis();
  float dt = currentTime - lastTime; // Time difference in milliseconds
  long deltaPosition = newPosition - oldPosition;
  float RPMActual = -(deltaPosition / (float)encoderCPR) * (60000.0 / dt);
  lastTime = currentTime;
  oldPosition = newPosition;

  // Compute the baselinePWM value based on commanded speed
  int basePWM = speedToPWM(RPMTarget);

  // Use the PID Controller to create a PWM adjustment
  int deltaPWM = PID.compute(RPMTarget, RPMActual, dt/1000); // Convert dt to seconds for PID computation
  int totalPWM = constrain(basePWM + deltaPWM, -255, 255); // Ensure PWM is within valid range
  sendMotorOutput(totalPWM);

  // Print the current state
  Serial.print("setpoint:");
  Serial.print(RPMTarget);
  Serial.print(" rpm:");
  Serial.print(RPMActual);
  Serial.print(" pwm:");
  Serial.print(totalPWM);
  Serial.print(" basePWM:");
  Serial.print(basePWM);
  Serial.print(" deltaPWM:");
  Serial.print(deltaPWM);
  Serial.print(" time:");
  Serial.println(lastTime);

  delay(2);
}