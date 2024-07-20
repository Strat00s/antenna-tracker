#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <SPI.h>

REDIRECT_STDOUT_TO(Serial);

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

int yaw_dir = 1; //1 - MIN is clockwise; -1 - MIN is counter-clockwise
int pitch_dir = 1;

uint32_t pitch_steps = 0;
uint32_t yaw_steps = 0;


void moveYaw() {

}
void movePitch() {

}

void Yaw2Min() {

}

void Yaw2Max() {

}

bool homePitch(uint32_t timeout = 30000) {

}

bool homeYaw(uint32_t timeout = 30000) {
    stepper_yaw.move(60000 * yaw_dir);
    unsigned long timer = millis();
    //move yaw in loop until endstop is hit
    while (digitalRead(ENDSTOP_YAW_MIN) && digitalRead(ENDSTOP_YAW_MAX) && millis() - timer < 30000) {
        stepper_yaw.setSpeed(500);
        stepper_yaw.runSpeedToPosition();
    }
    
    stepper_yaw.setCurrentPosition(0);
    stepper_yaw.disableOutputs();

    //flip direction if we moved clockwise instead of counter-clockwise and hit MIN endstop
    yaw_dir = !digitalRead(ENDSTOP_YAW_MAX) ? 1 : -1;

    printf("Endstop hit. New direction: %d\n", yaw_dir);


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

    printf("Total steps: %d\n", abs(stepper_yaw.currentPosition()));
    
    stepper_yaw.moveTo(abs(stepper_yaw.currentPosition()) / 2 * yaw_dir);
    while (stepper_yaw.distanceToGo()) {
        stepper_yaw.setSpeed(500);
        stepper_yaw.runSpeedToPosition();
    }

    pitch_steps = stepper_yaw.currentPosition();
    stepper_yaw.setCurrentPosition(0);
    stepper_yaw.disableOutputs();

    printf("Current position: %d\n", stepper_yaw.currentPosition());

    return true;
}


void setup() {
    // put your setup code here, to run once:
    pinMode(ENDSTOP_PITCH_MIN, INPUT_PULLUP);
    pinMode(ENDSTOP_YAW_MIN, INPUT_PULLUP);
    pinMode(ENDSTOP_YAW_MAX, INPUT_PULLUP);
    Serial1.begin(115200);
    printf("START\n");
    stepper_yaw.setMaxSpeed(1000);
    stepper_pitch.setMaxSpeed(1000);

    stepper_yaw.setSpeed(500);
    
    homeYaw();

    stepper_yaw.disableOutputs();
    stepper_pitch.disableOutputs();
}



void loop() {
    //stepper_yaw.setSpeed(200);
    //stepper_yaw.runSpeedToPosition();
    //printf("PITCH: %d | YAW_MAX: %d | YAM_MIN: %d | YAW: %d | PITCH: %d\n", digitalRead(ENDSTOP_PITCH_MIN), digitalRead(ENDSTOP_YAW_MAX), digitalRead(ENDSTOP_YAW_MIN), stepper_yaw.distanceToGo(), stepper_pitch.distanceToGo());
    // put your main code here, to run repeatedly:
}
