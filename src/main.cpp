#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <SPI.h>
#include <string>
#include <vector>
#include <FreeRTOS.h>
#include <task.h>

using namespace std;


/*---(PIN DEFINITIONS)---*/
//endstops
#define ENDSTOP_PITCH_MIN 13
#define ENDSTOP_YAW_MIN   10
#define ENDSTOP_YAW_MAX   1
//gps
#define GPS_TX
#define GPS_RX
//i2c
#define SDA
#define SCK
//motors
#define PITCH_N1 6
#define PITCH_N2 7
#define PITCH_N3 8
#define PITCH_N4 9
#define YAW_N1 2
#define YAW_N2 3
#define YAW_N3 4
#define YAW_N4 5
//buttons
#define FWD
#define BCK
#define OK
//comms
#define COMMS_TX
#define COMMS_RX


xTaskHandle gps_task;
xTaskHandle comms_task;
xTaskHandle stepper_task;


/*
//Read config
Home pitch
    Slam to end
    Go to endstop
    Measure how much was traveled
Home yaw
    Go to one endstop
    Go to sencond endstop
    Measure how much was traveled
Home
    Center yaw, look forward
Check if there is communication with PC/Radio for coordinations
    If not, manual operation only
Check if GPS is connected
    If so, use it as current HOME location
    If not, use HOME_XXX messages for current HOME location
Check if compass is connected
    If connected - nothing has to be done
    If not, user must rotate the tracker to face north
Check if baro is connected
    If so, use it as current HOME height
    If not, use HOME_XXX messages for current HOME height

Wait and read data from PC/Radio

*/

AccelStepper stepper_pitch(AccelStepper::FULL4WIRE, PITCH_N1, PITCH_N3, PITCH_N2, PITCH_N4);
AccelStepper stepper_yaw(AccelStepper::FULL4WIRE, YAW_N1, YAW_N3, YAW_N2, YAW_N4);
MultiStepper steppers;


int yaw_dir = 1;
int pitch_dir = 1;

bool yaw_calibrated = false;
bool pitch_calibrated = false;

uint32_t pitch_steps = 0;
uint32_t yaw_steps = 0;


void movePitchTo(int pos, int speed = 500) {
    stepper_pitch.moveTo(pos * pitch_dir);
    while (stepper_pitch.distanceToGo()) {
        stepper_pitch.setSpeed(speed);
        stepper_pitch.runSpeedToPosition();
    }
}

void moveYawTo(int pos, int speed = 500) {
    stepper_yaw.moveTo(pos * yaw_dir);
    while (stepper_yaw.distanceToGo()) {
        stepper_yaw.setSpeed(speed);
        stepper_yaw.runSpeedToPosition();
    }
}

void movePitchBy(int pos, int speed = 500) {
    stepper_pitch.move(pos * pitch_dir);
    while (stepper_pitch.distanceToGo()) {
        stepper_pitch.setSpeed(speed);
        stepper_pitch.runSpeedToPosition();
    }
}

void moveYawBy(int pos, int speed = 500) {
    stepper_yaw.move(pos * yaw_dir);
    while (stepper_yaw.distanceToGo()) {
        stepper_yaw.setSpeed(speed);
        stepper_yaw.runSpeedToPosition();
    }
}


void calibratePitch() {
    printf("Calibrating PITCH\n");
    stepper_pitch.move(4000 * pitch_dir);

    //move yaw in loop until endstop is hit
    while (stepper_pitch.distanceToGo()) {
        if (!digitalRead(ENDSTOP_PITCH_MIN) && stepper_pitch.distanceToGo() < 2700)
            break;

        stepper_pitch.setSpeed(700);
        stepper_pitch.runSpeedToPosition();
    }

    //wrong direction
    if (!digitalRead(ENDSTOP_PITCH_MIN)) {
        pitch_dir = -1;
        stepper_pitch.move(3000 * pitch_dir);
        while (stepper_pitch.distanceToGo()) {
            stepper_pitch.setSpeed(700);
            stepper_pitch.runSpeedToPosition();
        }
    }
    stepper_pitch.setCurrentPosition(0);

    stepper_pitch.move(-30000 * pitch_dir);
    while (digitalRead(ENDSTOP_PITCH_MIN)) {
        stepper_pitch.setSpeed(500);
        stepper_pitch.runSpeedToPosition();
    }

    pitch_steps = abs(stepper_pitch.currentPosition());
    stepper_pitch.setCurrentPosition(0);

    pitch_calibrated = true;
}

void calibrateYaw() {
    printf("Calibrating YAW\n");
    stepper_yaw.move(60000 * yaw_dir);

    //move yaw in loop until endstop is hit
    while (digitalRead(ENDSTOP_YAW_MIN) && digitalRead(ENDSTOP_YAW_MAX)) {
        stepper_yaw.setSpeed(500);
        stepper_yaw.runSpeedToPosition();
    }

    stepper_yaw.setCurrentPosition(0);

    //flip direction if we moved clockwise instead of counter-clockwise and hit MIN endstop
    yaw_dir = !digitalRead(ENDSTOP_YAW_MAX) ? 1 : -1;

    if (!digitalRead(ENDSTOP_YAW_MIN)) {
        stepper_yaw.move(60000 * yaw_dir);
        while (digitalRead(ENDSTOP_YAW_MAX)) {
            stepper_yaw.setSpeed(500);
            stepper_yaw.runSpeedToPosition();
        }
    }
    else if (!digitalRead(ENDSTOP_YAW_MAX)) {
        stepper_yaw.move(-60000 * yaw_dir);
        while (digitalRead(ENDSTOP_YAW_MIN)) {
            stepper_yaw.setSpeed(500);
            stepper_yaw.runSpeedToPosition();
        }
    }

    stepper_yaw.moveTo(abs(stepper_yaw.currentPosition()) / 2 * yaw_dir);
    while (stepper_yaw.distanceToGo()) {
        stepper_yaw.setSpeed(500);
        stepper_yaw.runSpeedToPosition();
    }

    pitch_steps = stepper_yaw.currentPosition();
    stepper_yaw.setCurrentPosition(0);

    yaw_calibrated = true;
}

void homePitch() {
    if (!pitch_calibrated)
        calibratePitch();
    movePitchTo(0);
}

void homeYaw() {
    if (!yaw_calibrated)
        calibrateYaw();
    moveYawTo(0);
}


void gpsTask(void *) {
    while (true) {
        printf("GPS CORE ID: %d\n", rp2040.cpuid());
        delay(500);
    }
}

void commsTask(void *) {
    string in = "";
    while (true) {
        if (Serial.available() > 0) {
            int c = Serial.read();
            if (c < 0) {
                printf("\nFailed to read input!\n");
                continue;
            }
            if (c != '\n' && c != '\r') {
                Serial.print(char(c));
                in += c;
                continue;
            }

            if (!in.length())
                continue;

            printf("\n");
            size_t split = in.find(' ');
            string cmd = in;
            string arg = "";

            //cmd without argument
            if (split != string::npos) {
                cmd = in.substr(0, split);
                arg = in.substr(split + 1);
            }

            if (cmd == "UAV_LAT") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else if (cmd == "UAV_LON") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else if (cmd == "UAV_ALT") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else if (cmd == "ARMED") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else if (cmd == "HOME_LAT") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else if (cmd == "HOME_LON") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else if (cmd == "HOME_ALT") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else if (cmd == "MANUAL_YAW") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else if (cmd == "MANUAL_PITCH") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else if (cmd == "HOME") {
                printf("NOT YET IMPLEMENTED\n");
            }
            else {
               printf("UNKNOWN COMMAND\n");
            }

            in.clear();
        }
    }
}

void stepperTask(void *) {
    while (true) {
        printf("STEPPER CORE ID: %d\n", rp2040.cpuid());
        delay(500);
    }
}


void setup() {
    // put your setup code here, to run once:
    pinMode(ENDSTOP_PITCH_MIN, INPUT_PULLUP);
    pinMode(ENDSTOP_YAW_MIN, INPUT_PULLUP);
    pinMode(ENDSTOP_YAW_MAX, INPUT_PULLUP);
    Serial.begin(115200);

    stepper_yaw.setMaxSpeed(1000);
    stepper_pitch.setMaxSpeed(1000);

    //xTaskCreate(gpsTask, "GPS", 2048, nullptr, 4, &gps_task);
    xTaskCreate(commsTask, "COMMS", 1024, nullptr, 1, &comms_task);
    //xTaskCreate(stepperTask, "STEPPER", 2048, nullptr, 0, &stepper_task);

    //homePitch();
    //homeYaw();

    stepper_yaw.disableOutputs();
    stepper_pitch.disableOutputs();

    vTaskDelete(nullptr);
}



void loop() {
    //stepper_yaw.setSpeed(200);
    //stepper_yaw.runSpeedToPosition();
    //printf("PITCH: %d | YAW_MAX: %d | YAM_MIN: %d | YAW: %d | PITCH: %d\n", digitalRead(ENDSTOP_PITCH_MIN), digitalRead(ENDSTOP_YAW_MAX), digitalRead(ENDSTOP_YAW_MIN), stepper_yaw.distanceToGo(), stepper_pitch.distanceToGo());
    // put your main code here, to run repeatedly:
}
