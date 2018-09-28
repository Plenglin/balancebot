#define LOGLEVEL_INFO

#define INPUT_BUFFER_SIZE 100

#define PIN_SONAR_TRIG 3
#define PIN_SONAR_ECHO 2

#define PIN_IMU_INT 3

#define PIN_LEFT_STP 10
#define PIN_LEFT_DIR 10
#define PIN_LEFT_EN 10

#define PIN_RIGHT_STP 10
#define PIN_RIGHT_DIR 10
#define PIN_RIGHT_EN 10

#define COM_BAUD 115200

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include "stepstick.hpp"
#include "pid.hpp"
#include "log.h"
#include "params.h"

volatile bool cmdReady;
volatile bool mpuReady;
volatile bool sonarReady;

uint16_t echoParams = 0;

String inputBuffer = "";

uint8_t fifoBuffer[64];
MPU6050 mpu;
Quaternion rotation;
int sonarWidth = 0;

StepStick right(PIN_RIGHT_EN, PIN_RIGHT_STP, PIN_RIGHT_DIR, 10, true);
StepStick left(PIN_LEFT_EN, PIN_LEFT_STP, PIN_LEFT_DIR, 10, true);

PID pidSteps(1, 0, 0);
PID pidPitch(1, 0, 0);
PID pidYaw(1, 0, 0);

void onEchoInterrupt() {
    sonarReady = true;
}

void onIMUInterrupt() {
    mpuReady = true;
}

void serialEvent() {
    int i = 0;
    while (Serial.available()) {
        if (i >= INPUT_BUFFER_SIZE) continue;
        char c = static_cast<char>(Serial.read());
        inputBuffer[i++] = c;
    }
    cmdReady = true;
}

void writeRotationData() {
    if (echoParams & PARAM_ECHO_ROTATION) {
        Serial.println(ARG_IMU + String(rotation.w) + ";" + rotation.x + ";" + rotation.y + ";" + rotation.z);
    }
}

void writeSonarData() {
    if (echoParams & PARAM_ECHO_SONAR) {
        Serial.println(ARG_SONAR + sonarWidth);
    }
}

void setup() {
    Serial.begin(COM_BAUD);

    inputBuffer.reserve(INPUT_BUFFER_SIZE);

    attachInterrupt(PIN_SONAR_ECHO, onEchoInterrupt, RISING);
    attachInterrupt(PIN_IMU_INT, onIMUInterrupt, RISING);

    mpu.setInterruptMode(true);
    int status = mpu.dmpInitialize();
    if (status == 0) {
        mpu.setDMPEnabled(true);
    } else {
        LOG_E("DMP failed to initialize! Error code " + status);
        while (true);
    }
}

void loop() {
    if (mpuReady) {
        mpu.dmpGetQuaternion(&rotation, fifoBuffer);
        mpuReady = false;
        writeRotationData();
    }

    if (sonarReady) {
        sonarWidth = pulseIn(PIN_SONAR_ECHO, true);
        digitalWrite(PIN_SONAR_TRIG, true);
        delay(1);
        digitalWrite(PIN_SONAR_TRIG, false);
        sonarReady = false;
        writeSonarData();
    }

    if (cmdReady) {
        cmdReady = false;
    }
    
}