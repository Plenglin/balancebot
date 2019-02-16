#define LOGLEVEL_DEBUG

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#include "stepstick.hpp"
#include "pid.hpp"

#include "constants.hpp"
#include "ioconstants.hpp"
#include "log.hpp"

volatile bool cmdReady = false;
volatile bool imuReady = false;
volatile bool sonarReady = false;

uint8_t echoParams = PARAM_ECHO_ROTATION;

char inputCommand;
String argBuffer = "";

uint8_t fifoBuffer[64];

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(21234);
int sonarWidth = 0;

unsigned long lastRotationUpdate = 0;

StepStick left(PIN_LEFT_EN, PIN_LEFT_STP, PIN_LEFT_DIR, STEPPER_PPR, true, 1000, 200);
StepStick right(PIN_RIGHT_EN, PIN_RIGHT_STP, PIN_RIGHT_DIR, STEPPER_PPR, false, 1000, 200);

PID pidSteps(0, 0, 0);
PID pidPitch(200, 0, 0);
PID pidYaw(0, 0, 0);

float pitch;  // degrees

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
        //fixed aRawX(A_X), aRawZ(A_Z);
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

    /* Initialise the sensors */
    LOG_D("Setting up accelerometer")
    if(!accel.begin()) {
        /* There was a problem detecting the ADXL345 ... check your connections */
        LOG_F("Ooops, no LSM303 detected ... Check your wiring!");
        while(1) delay(1000);
    }

    LOG_D("Setting up magnetometer")
    if(!mag.begin()) {
        /* There was a problem detecting the LSM303 ... check your connections */
        LOG_F("Ooops, no LSM303 detected ... Check your wiring!");
        while(1) delay(1000);
    }
    
    LOG_D("Setting up gyro")
    if(!gyro.begin(GYRO_RANGE_2000DPS)) {
        /* There was a problem detecting the L3GD20 ... check your connections */
        LOG_F("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
        while(1) delay(1000);
    }


    LOG_D("Enabling stepper motors")
    left.setEnabled(true);
    right.setEnabled(true);

    pidPitch.setTarget(0);

    LOG_D("Setting up timer interrupts")
    // 1000Hz http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
    // TIMER 1 for interrupt frequency 1000 Hz:
    cli(); // stop interrupts
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1  = 0; // initialize counter value to 0
    // set compare match register for 1000 Hz increments
    OCR1A = 15999; // = 16000000 / (1 * 1000) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12, CS11 and CS10 bits for 1 prescaler
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei(); // allow interrupts
}

void updateIMU() {
    
    /* Get a new sensor event */
    sensors_event_t evAccel;
    sensors_event_t evMag;
    sensors_event_t evGyro;
    
    /* Display the results (acceleration is measured in m/s^2) */
    accel.getEvent(&evAccel);
    mag.getEvent(&evMag);
    gyro.getEvent(&evGyro);

    unsigned long currentTime = micros();
    unsigned long dtLong = currentTime - lastRotationUpdate;
    lastRotationUpdate = currentTime;
    float dt = dtLong / 1000000.0;

    float gy = -evGyro.gyro.y * dt;
    float accAngle = atan2(-evAccel.acceleration.z, evAccel.acceleration.x);
    pitch = (pitch + gy) * 0.9975 + accAngle * 0.0025;

    float degPitch = degrees(pitch);

    writeRotationData();
    float outPitch = pidPitch.push(degPitch, dt);
    LOG_D(String(dt) + "\t" + String(degPitch) + "\t" + String(accAngle));
    left.setTargetVelocity((long) outPitch);
    right.setTargetVelocity((long) outPitch);
    //LOG_D(currentTime)
}

void loop() {
    #ifndef JANKY_PIN_13
    digitalWrite(13, (millis() % 1000) > 500);
    #endif
    //LOG_D(left.getStepCount())
    
    updateIMU();
}

ISR(TIMER1_COMPA_vect) {
    left.update();
    right.update();
}
