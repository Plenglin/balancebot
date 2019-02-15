#define LOGLEVEL_DEBUG

#include <Arduino.h>
#include <MPU6050.h>

#include "stepstick.hpp"
#include "pid.hpp"

#include "constants.hpp"
#include "ioconstants.hpp"
#include "log.hpp"

// if reorientation is necessary
#define A_X -aRawZ
#define A_Y aRawY
#define A_Z aRawX
#define G_X -gRawZ
#define G_Y gRawY
#define G_Z gRawX

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

StepStick left(PIN_LEFT_EN, PIN_LEFT_STP, PIN_LEFT_DIR, STEPPER_PPR, true, 1000, 200);
StepStick right(PIN_RIGHT_EN, PIN_RIGHT_STP, PIN_RIGHT_DIR, STEPPER_PPR, false, 1000, 200);

PID pidSteps(0, 0, 0);
PID pidPitch(300, 0, 0);
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

    mpu.initialize();
    mpu.setYGyroOffset(0);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu.setRate(100);
    mpu.setIntEnabled(true);
    mpu.setIntDataReadyEnabled(true);

    left.setEnabled(true);
    right.setEnabled(true);

    pidPitch.setTarget(0);

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
    //LOG_D("updating data from IMU")

    mpu.getAcceleration(&aRawX, &aRawY, &aRawZ);
    mpu.getRotation(&gRawX, &gRawY, &gRawZ);

    //fPitch.update(A_X / A_Y, appxTan.getUpper());
    unsigned long currentTime = micros();
    float dt = (currentTime - lastRotationUpdate) / 1000000.0;
    lastRotationUpdate = currentTime;

    float gy = (G_Y - 8.69) * 0.0152588 * dt;  // the proper number should be 0.030517578 but idk why I have to div by 2
    float ax = A_X;
    float az = A_Z;
    //float ratio = -ax / az;
    float accAngle = -57.29577795 * atan2(ax, az) - 11.12;
    pitch = (pitch + gy) * 0.9975 + accAngle * 0.0025;
    //pitch = accAngle;
    pitch += gy;

    writeRotationData();
    float outPitch = pidPitch.push(pitch, dt);
    LOG_D(String(currentTime) + "\t" + String(pitch) + "\t" + String(accAngle));
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
