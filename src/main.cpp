#define LOGLEVEL_DEBUG

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#include "fixedpoint.hpp"
#include "complementaryfilter.hpp"
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

StepStick left(PIN_LEFT_EN, PIN_LEFT_STP, PIN_LEFT_DIR, STEPPER_PPR, true, 1500);
StepStick right(PIN_RIGHT_EN, PIN_RIGHT_STP, PIN_RIGHT_DIR, STEPPER_PPR, false, 1500);

PID pidSteps(0, 0, 0);
PID pidPitch(fixed(3000), fixed(0), fixed(0));
PID pidYaw(0, 0, 0);

fixed pitch;  // millidegrees? idk wtf these are

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
    int dt = (int) (currentTime - lastRotationUpdate);
    lastRotationUpdate = currentTime;

    fixed gy(G_Y), ax(A_X), az(A_Z);
    gy *= fixed(2) * dt / 1000;
    fixed ratio = -ax / az;
    fixed accAngle = atan(ratio);
    pitch = (pitch + gy) * fixed(250) + accAngle * fixed(4468);

    writeRotationData();
    long outPitch = -pidPitch.pushError(pitch, dt).getValue() >> 8;
    LOG_D(pitch.toString() + "\t" + String(outPitch));
    left.setTargetVelocity(outPitch);
    right.setTargetVelocity(outPitch);
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
    left.setEnabled(true);
    right.setEnabled(true);
    left.setTargetVelocity(3000);
    //right.setVelocity(3000);
}

void loop() {
    digitalWrite(13, (millis() % 1000) > 500);

    updateIMU();

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
