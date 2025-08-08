#include <Arduino.h>
#undef min // Was getting errors about min and max definition conflicts, so undefined them after the Arduino include
#undef max
#include <Encoder.h>
#include "PIDController.h"
#include <vector>
#include <iostream>
#include <cmath>



// Define pin variables
const int INA1 = 2;
const int INB1 = 4;
const int PWM_PIN1 = 3;

const int INA2 = 5;
const int INB2 = 7;
const int PWM_PIN2 = 6;

const int INA3 = 8;
const int INB3 = 10;
const int PWM_PIN3 = 9;

// Define minimum, maximum speed and commanded speed (manually update these while testing)
const int minSpeed = -530; // Minimum speed in RPM
const int maxSpeed = 530; // Maximum speed in RPM
// For now, manuall set the rover speed in the format {yaw rate, x velocity, y velocity}
std::vector<float> roverSpeed = {0.1f, 0.0f, 12.0f}; 

// Define variables for encoder reading and initialize object
const int encoderPinA1 = 48; // Must be interrupt capable; keep in mind if you change to another board
const int encoderPinB1 = 49;

const int encoderPinA2 = 50; // Must be interrupt capable; keep in mind if you change to another board
const int encoderPinB2 = 51;

const int encoderPinA3 = 52; // Must be interrupt capable; keep in mind if you change to another board
const int encoderPinB3 = 53;

Encoder Enc1(encoderPinA1, encoderPinB1);
Encoder Enc2(encoderPinA2, encoderPinB2);
Encoder Enc3(encoderPinA3, encoderPinB3);

// Define values for converting the encoder readings to speed (RPM)
const int encoderCPR = 1200; // Counts per revolution of the encoder (output shaft)
unsigned long lastTime = 0;

// Initialize the PID controller object
// Note: PID controller controls rate of change of PWM signal, not speed directly, which is why the maximum and minimum outputs are set as they are
// First value is proportional gain, second is integral gain, third is derivative gain
const int PID_K = 6.5; // Proportional gain
const int PID_I = 4.0; // Integral gain
const int PID_D = 0.0; // Derivative gain

PIDController PID1(PID_K, PID_I, PID_D, -255, 255); // PID gains and output limits
PIDController PID2(PID_K, PID_I, PID_D, -255, 255);
PIDController PID3(PID_K, PID_I, PID_D, -255, 255); // May not need three separate controllers, but making this initially



// Define a function to convert from rotational speed to PWM output
// Note: negative PWM values will not actually be output as negative PWM values, but will simply reverse the motor direction and use the magnitue for speed
int speedToPWM(int commandedSpeed) {
  // Assuming speed is in RPM and we want to convert it to a PWM value
  return map(commandedSpeed, minSpeed, maxSpeed, -255, 255);
}

// Define a function to send a motor output
void sendMotorOutput(const std::vector<int>& speeds) {
    const int INA[3] = {INA1, INA2, INA3};
    const int INB[3] = {INB1, INB2, INB3};
    const int PWM_PIN[3] = {PWM_PIN1, PWM_PIN2, PWM_PIN3};

    for (int i = 0; i < 3; ++i) {
        if (speeds[i] == 0) {
            digitalWrite(INA[i], LOW);
            digitalWrite(INB[i], LOW);
        } else {
            digitalWrite(INA[i], speeds[i] > 0 ? HIGH : LOW);
            digitalWrite(INB[i], speeds[i] < 0 ? HIGH : LOW);
            analogWrite(PWM_PIN[i], abs(speeds[i]));
        }
    }
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



void setup() {
  Serial.begin(115200);

  // Initialize pins for motor control
  pinMode(INA1, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(PWM_PIN1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(INB2, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);
  pinMode(INA3, OUTPUT);
  pinMode(INB3, OUTPUT);
  pinMode(PWM_PIN3, OUTPUT);
  Serial.println("Motor Control Initialized");
}



long oldPosition1  = 0;
long oldPosition2  = 0;
long oldPosition3  = 0;

void loop() {
  // Read the encoder position and calculate speed
  long newPosition1 = Enc1.read();
  long newPosition2 = Enc2.read();
  long newPosition3 = Enc3.read();
  unsigned long currentTime = millis();
  float dt = currentTime - lastTime; // Time difference in milliseconds
  long deltaPosition1 = newPosition1 - oldPosition1;
  long deltaPosition2 = newPosition2 - oldPosition2;
  long deltaPosition3 = newPosition3 - oldPosition3;
  float RPM1Actual = -(deltaPosition1 / (float)encoderCPR) * (60000.0 / dt);
  float RPM2Actual = -(deltaPosition2 / (float)encoderCPR) * (60000.0 / dt);
  float RPM3Actual = -(deltaPosition3 / (float)encoderCPR) * (60000.0 / dt);
  // Update time and store old positions
  lastTime = currentTime;
  oldPosition1 = newPosition1;
  oldPosition2 = newPosition2;
  oldPosition3 = newPosition3;

  // Calculate the commanded speed based on rover speed
  float yawRate = roverSpeed[0];
  float xVelocity = roverSpeed[1];
  float yVelocity = roverSpeed[2];
  std::vector<float> commandedSpeeds = kiwiDriveInverseKinematics(yawRate, xVelocity, yVelocity);

  int PWM1Base = speedToPWM(static_cast<int>(commandedSpeeds[0]));
  int PWM2Base = speedToPWM(static_cast<int>(commandedSpeeds[1]));
  int PWM3Base = speedToPWM(static_cast<int>(commandedSpeeds[2]));

  // Use the PID Controller to create a PWM adjustment
  int deltaPWM1 = PID1.compute(commandedSpeeds[0], RPM1Actual, dt/1000); // Convert dt to seconds for PID computation
  int totalPWM1 = constrain(PWM1Base + deltaPWM1, -255, 255); // Ensure PWM is within valid range
  int deltaPWM2 = PID2.compute(commandedSpeeds[1], RPM2Actual, dt/1000);
  int totalPWM2 = constrain(PWM2Base + deltaPWM2, -255, 255);
  int deltaPWM3 = PID3.compute(commandedSpeeds[2], RPM3Actual, dt/1000);
  int totalPWM3 = constrain(PWM3Base + deltaPWM3, -255, 255);

  // Send the motor output
  sendMotorOutput({totalPWM1, totalPWM2, totalPWM3});

  // Print the current state for Motor 1 for debugging
  Serial.print("setpoint:");
  Serial.print(commandedSpeeds[0]);
  Serial.print(" rpm:");
  Serial.print(RPM1Actual);
  Serial.print(" pwm:");
  Serial.print(totalPWM1);
  Serial.print(" basePWM:");
  Serial.print(PWM1Base);
  Serial.print(" deltaPWM:");
  Serial.print(deltaPWM1);
  Serial.print(" time:");
  Serial.println(lastTime);

  delay(2);
}