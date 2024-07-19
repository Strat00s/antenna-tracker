#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>


/*---(PIN DEFINITIONS)---*/
//endstops
#define ENDSTOP_PITCH_MIN 0
#define ENDSTOP_YAW_MIN   1
#define ENDSTOP_YAW_MAX   2
//gps
#define GPS_TX
#define GPS_RX
//i2c
#define SDA
#define SCK
//motors
#define PITCH_N1 1
#define PITCH_N2 2
#define PITCH_N3 3
#define PITCH_N4 4
#define YAW_N1 5
#define YAW_N2 6
#define YAW_N3 7
#define YAW_N4 8
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

AccelStepper stepper_pitch(AccelStepper::FULL4WIRE, PITCH_N1, PITCH_N2, PITCH_N3, PITCH_N4);
AccelStepper stepper_yaw(AccelStepper::FULL4WIRE, YAW_N1, YAW_N2, YAW_N3, YAW_N4);
MultiStepper steppers;

bool yaw_dir = 0; //0 - MIN is clockwise; 1 - MIN is counter-clockwise
bool pitch_dir = 0;

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

bool homeYaw(uint32_t timeout = 30000) {
    //move yaw in loop until endstop is hit
    while (!digitalRead(ENDSTOP_YAW_MIN) && !digitalRead(ENDSTOP_YAW_MAX)) {
        stepper_pitch.setSpeed(500);
    }

    //flip direction if we moved clockwise instead of counter-clockwise and hit MIN endstop
    yaw_dir = digitalRead(ENDSTOP_YAW_MIN);
    stepper_pitch.setCurrentPosition(0);
}

bool homePitch(uint32_t timeout = 30000) {

}


void setup() {
    // put your setup code here, to run once:
    Serial1.begin(115200);
    Serial2.begin(115200);
}

void loop() {
    // put your main code here, to run repeatedly:
}
