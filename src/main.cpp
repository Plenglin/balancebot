#define LOGLEVEL_DEBUG

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#include "fixedpoint.hpp"
#include "stepstick.hpp"
#include "pid.hpp"

#include "constants.hpp"
#include "ioconstants.hpp"
#include "log.hpp"

// if reorientation is necessary
#define A_X aRawX
#define A_Y aRawY
#define A_Z aRawZ
#define G_X gRawX
#define G_Y gRawY
#define G_Z gRawZ

volatile bool cmdReady = false;
volatile bool imuReady = false;
volatile bool sonarReady = false;

uint8_t echoParams = PARAM_ECHO_ROTATION;

char inputCommand;
String argBuffer = "";

uint8_t fifoBuffer[64];
int16_t aRawX, aRawY, aRawZ, gRawX, gRawY, gRawZ;
MPU6050 mpu;
int sonarWidth = 0;

unsigned long lastRotationUpdate = 0;

StepStick left(PIN_LEFT_EN, PIN_LEFT_STP, PIN_LEFT_DIR, STEPPER_PPR, true, 1000);
StepStick right(PIN_RIGHT_EN, PIN_RIGHT_STP, PIN_RIGHT_DIR, STEPPER_PPR, false, 1000);

PID pidSteps(0, 0, 0);
PID pidPitch(100, 0, 0);
PID pidYaw(0, 0, 0);

float pitch;  // millidegrees? idk wtf these are

void onEchoInterrupt() {
    sonarReady = true;
}

void onIMUInterrupt() {
    imuReady = true;
}

void serialEvent() {
    int i = 0;
    inputCommand = static_cast<char>(Serial.read());
    while (Serial.available() && i < INPUT_BUFFER_SIZE) {
        if (i >= INPUT_BUFFER_SIZE) continue;
        char c = static_cast<char>(Serial.read());
        if (c == '\n') break;  // split by newlines
        argBuffer[i++] = c;
    }
    cmdReady = true;
}

void writeRotationData() {
    if (echoParams & PARAM_ECHO_ROTATION) {
        fixed aRawX(A_X), aRawZ(A_Z);
    }
}

void writeSonarData() {
    if (echoParams & PARAM_ECHO_SONAR) {
        Serial.println(OUT_ARG_SONAR + sonarWidth);
    }
}

void updateIMU() {
    //LOG_D("updating data from IMU")

    mpu.getAcceleration(&aRawX, &aRawY, &aRawZ);
    mpu.getRotation(&gRawX, &gRawY, &gRawZ);

    //fPitch.update(A_X / A_Y, appxTan.getUpper());
    unsigned long currentTime = micros();
    unsigned long dt = currentTime - lastRotationUpdate;
    lastRotationUpdate = currentTime;

    //fixed gy(G_Y), ax(A_X), az(A_Z);
    float gy = G_Y * 0.0305176 * dt / 1000000;
    float ax = A_X;
    float az = A_Z;
    //float ratio = -ax / az;
    float accAngle = atan2(az, ax) * 57.29577794;
    pitch = (pitch + gy) * 0.995 + accAngle * 0.005;
    //pitch = accAngle;

    writeRotationData();
    long outPitch = -pidPitch.pushError(pitch + 90, dt);
    LOG_D(String(pitch) + "\t" + String(outPitch));
    left.setTargetVelocity(outPitch);
    right.setTargetVelocity(outPitch);
    LOG_D(currentTime)
}

void setup() {
    Serial.begin(COM_BAUD);

    LOG_I("Initializing...")

    #ifdef JANKY_PIN_13
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    #endif

    argBuffer.reserve(INPUT_BUFFER_SIZE);

    pinMode(PIN_SONAR_ECHO, INPUT);
    pinMode(PIN_IMU_INT, INPUT);

    mpu.initialize();
    mpu.setYGyroOffset(0);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu.setRate(100);
    mpu.setIntEnabled(true);
    mpu.setIntDataReadyEnabled(true);
    //attachInterrupt(PIN_SONAR_ECHO, onEchoInterrupt, RISING);
    //attachInterrupt(PIN_IMU_INT, onIMUInterrupt, FALLING);
    left.setEnabled(false);
    right.setEnabled(false);
    //left.setTargetVelocity(10000);
    //right.setVelocity(3000);
}

void loop() {
    digitalWrite(13, (millis() % 1000) > 500);

    //updateIMU();

    /*if (sonarReady) {
        LOG_D("received sonar interrupt")
        sonarWidth = pulseIn(PIN_SONAR_ECHO, true);
        digitalWrite(PIN_SONAR_TRIG, true);
        delayMicroseconds(SONAR_MIN_DELAY);
        digitalWrite(PIN_SONAR_TRIG, false);
        sonarReady = false;
        writeSonarData();
    }*/

    if (cmdReady) {
        switch (inputCommand) {
            case IN_ARG_SET_ECHO_PARAMS:
                echoParams = argBuffer.charAt(0);
                break;
            case IN_ARG_SET_PITCH:
                pidPitch.setTarget(argBuffer.toInt());
                break;
            case IN_ARG_SET_YAW: 
                pidYaw.setTarget(argBuffer.toInt());
                break;
        }
        Serial.println(OUT_ARG_ACK);
        cmdReady = false;
    }

    left.update();
    right.update();
}
