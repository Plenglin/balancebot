#define LOGLEVEL_DEBUG

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#include "stepstick.hpp"
#include "pid.hpp"

#include "constants.hpp"
#include "ioconstants.hpp"
#include "log.hpp"

volatile bool cmdReady = false;
volatile bool mpuReady = false;
volatile bool sonarReady = false;

uint8_t echoParams = PARAM_ECHO_ROTATION;

char inputCommand;
String argBuffer = "";

uint8_t fifoBuffer[64];
float ypr[3] = {0};
//VectorFloat gravity;
MPU6050 mpu;
//Quaternion rotation;
int sonarWidth = 0;

unsigned long lastRotationUpdate = 0;

StepStick right(PIN_RIGHT_EN, PIN_RIGHT_STP, PIN_RIGHT_DIR, STEPPER_PPR);
StepStick left(PIN_LEFT_EN, PIN_LEFT_STP, PIN_LEFT_DIR, STEPPER_PPR);

PID pidSteps(0, 0, 0);
PID pidPitch(100, 0, 0);
PID pidYaw(0, 0, 0);

void onEchoInterrupt() {
    sonarReady = true;
}

void onIMUInterrupt() {
    mpuReady = true;
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
        //Serial.println(OUT_ARG_IMU + String(rotation.w) + ";" + rotation.x + ";" + rotation.y + ";" + rotation.z);
    }
}

void writeSonarData() {
    if (echoParams & PARAM_ECHO_SONAR) {
        Serial.println(OUT_ARG_SONAR + sonarWidth);
    }
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

    mpu.setInterruptMode(true);
    //attachInterrupt(PIN_SONAR_ECHO, onEchoInterrupt, RISING);
    attachInterrupt(PIN_IMU_INT, onIMUInterrupt, FALLING);
}

void loop() {
    digitalWrite(13, (millis() % 1000) > 500);

    int x, y, z;
    LOG_D("asdf")
    mpu.getAcceleration(&x, &y, &z);
    LOG_D(x)
    if (mpuReady) {
        LOG_D("received MPU interrupt")
        //mpu.dmpGetQuaternion(&rotation, fifoBuffer);
        //mpu.dmpGetGravity(&gravity, &rotation);
        //mpu.dmpGetYawPitchRoll(ypr, &rotation, &gravity);

        unsigned long currentTime = micros();
        unsigned long delta = currentTime - lastRotationUpdate;
        lastRotationUpdate = currentTime;

        long outYaw = 0;//pidYaw.pushError(0, delta);
        long outPitch = pidPitch.pushError(ypr[1] - 0, delta);
        left.setVelocity(outYaw + outPitch);
        right.setVelocity(outYaw - outPitch);
        mpuReady = false;
        writeRotationData();
    }

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

    //left.update();
    //right.update();
}
