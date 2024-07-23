#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <SPI.h>
#include <string>
#include <vector>
#include <deque>
#include <tuple>
#include <set>
#include <FreeRTOS.h>
#include <task.h>
#include "threadsafe_deque.hpp"
#include "gps_manager.hpp"

using namespace std;


/*---(PIN DEFINITIONS)---*/
//endstops
#define ENDSTOP_PITCH_MIN 13
#define ENDSTOP_YAW_MIN   10
#define ENDSTOP_YAW_MAX   1
//gps
#define GPS_SERIAL Serial2
#define GPS_TX 21
#define GPS_RX 20
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

#define PITCH_RATIO 50 / 14
//#define YAW_RATIO 60 / 20
#define YAW_RATIO 60 / 25

#define deg2step(deg, ratio) (deg * 32 * 63.68395 / 360 * ratio)


//TODO button controlls
//TODO display
//TODO GPS config (glonass, gps, beidu, galileo)


xTaskHandle gps_task;
xTaskHandle comms_task;
xTaskHandle stepper_task;
xTaskHandle serial_out_task;


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

bool yaw_homed = false;
bool pitch_homed = false;

long pitch_steps = 999999;
long yaw_steps   = 999999;



GPSManager gps_manager;


/*
void movePitch(int speed = 500) {
    if (!pitch_homed) {
        printf("Need to home PITCH axis first!\n");
        return;
    }
    while (stepper_pitch.distanceToGo()) {
        if (stepper_pitch.distanceToGo() * pitch_dir < 0 && !digitalRead(ENDSTOP_PITCH_MIN))
            break;
        if (stepper_pitch.distanceToGo() * pitch_dir > 0 && stepper_pitch.currentPosition() * pitch_dir >= pitch_steps)
            break;
        stepper_pitch.setSpeed(speed);
        stepper_pitch.runSpeedToPosition();
    }
    printf("PITCH: %d\n", stepper_pitch.currentPosition());
}

void moveYaw(int speed = 500) {
    if (!yaw_homed) {
        printf("Need to home YAW axis first!\n");
        return;
    }
    while (stepper_yaw.distanceToGo()) {
        if ((stepper_yaw.distanceToGo() * yaw_dir > 0 && !digitalRead(ENDSTOP_YAW_MAX)) || (stepper_yaw.distanceToGo() * yaw_dir < 0 && !digitalRead(ENDSTOP_YAW_MIN)))
            break;
        stepper_yaw.setSpeed(speed);
        stepper_yaw.runSpeedToPosition();
    }
    printf("YAW: %d\n", stepper_yaw.currentPosition());
}
*/

/*
void movePitchTo(int pos, int speed = 500) {
    stepper_pitch.moveTo(pos * pitch_dir);
    movePitch(speed);
}

void moveYawTo(int pos, int speed = 500) {
    stepper_yaw.moveTo(pos * yaw_dir);
    moveYaw(speed);
}
*/

/*
void movePitchBy(int pos, int speed = 500) {
    stepper_pitch.move(pos * pitch_dir);
    movePitch(speed);
}

void moveYawBy(int pos, int speed = 500) {
    stepper_yaw.move(pos * yaw_dir);
        moveYaw(speed);

}
*/

/*
void calibratePitch() {
    printf("Calibrating PITCH\n");
    stepper_pitch.move(4000 * pitch_dir);
    int togo_orig = stepper_yaw.distanceToGo();
    //move yaw in loop until endstop is hit
    while (stepper_pitch.distanceToGo()) {
        if (!digitalRead(ENDSTOP_PITCH_MIN) && abs(togo_orig - stepper_yaw.distanceToGo()) > 300)
            break;
        stepper_pitch.setSpeed(700);
        stepper_pitch.runSpeedToPosition();
    }


    //wrong direction
    if (!digitalRead(ENDSTOP_PITCH_MIN)) {
        pitch_dir = -1;
        movePitchBy(4000);
    }
    stepper_pitch.setCurrentPosition(0);
    movePitchBy(-30000);
    pitch_steps = abs(stepper_pitch.currentPosition());
    stepper_pitch.setCurrentPosition(0);
    pitch_calibrated = true;
}

void calibrateYaw() {
    printf("Calibrating YAW\n");
    stepper_yaw.move(60000 * yaw_dir);
    int togo_orig = stepper_yaw.distanceToGo();
    //move yaw in loop until endstop is hit
    while (stepper_yaw.distanceToGo()) {
        if ((!digitalRead(ENDSTOP_YAW_MIN) || !digitalRead(ENDSTOP_YAW_MAX)) && abs(togo_orig - stepper_yaw.distanceToGo()) > 300)
            break;
        stepper_yaw.setSpeed(500);
        stepper_yaw.runSpeedToPosition();
    }
    
    stepper_yaw.setCurrentPosition(0);

    //flip direction if we moved clockwise instead of counter-clockwise and hit MIN endstop
    yaw_dir = !digitalRead(ENDSTOP_YAW_MAX) ? 1 : -1;

    if (!digitalRead(ENDSTOP_YAW_MIN))
        moveYawBy(60000);
    else if (!digitalRead(ENDSTOP_YAW_MAX))
        moveYawBy(-60000);
    yaw_steps = abs(stepper_yaw.currentPosition());
    stepper_yaw.setCurrentPosition(yaw_steps / 2 * yaw_dir);
    yaw_calibrated = true;
}
*/

/*
void homePitch() {
    pitch_homed = true;
    if (!pitch_calibrated)
        calibratePitch();
    else
        movePitchBy(-9999999);
    stepper_pitch.setCurrentPosition(0);
    movePitchTo(0);
}

void homeYaw() {
    yaw_homed = true;
    if (!yaw_calibrated)
        calibrateYaw();
    else
        moveYawBy(9999999);
    stepper_yaw.setCurrentPosition(yaw_steps / 2 * yaw_dir);
    moveYawTo(0);
}
*/

void stepperTask(void *) {
    while (true) {
        stepper_pitch.runSpeedToPosition();
        stepper_yaw.runSpeedToPosition();
    }
}

void gpsTask(void *) {
    GPS_SERIAL.setTX(GPS_RX);
    GPS_SERIAL.setRX(GPS_TX);
    GPS_SERIAL.setFIFOSize(256);
    int ret = gps_manager.begin(&GPS_SERIAL, 115200);
    if (ret) {
        printf("Failed to change baud rate: %d\n", ret);
        //TODO report it somewhere
    }


    deque<tuple<uint8_t, uint8_t, vector<uint8_t>>> msg_ubx_q;
    //msg_ubx_q.push_back({0x06, 0x17,{0, 0x4b, 0, 0b00001000, 0, 0, 0, 0, 1, 0, 0, 0x01, 0, 0, 0, 0, 0, 0, 0, 0}});
    //msg_ubx_q.push_back({0x06, 0x3E, {}});
    //msg_ubx_q.push_back({0x06, 0x02, {0, 0, 0, 0, 0b00011111, 1, 0, 0, 0, 0}});
    
    unsigned long timer = millis();

    while (true) {
        if (gps_manager.read()) {
            auto last_msg = gps_manager.getLastMsg();
            Serial.printf("%d %02x %02x %d | ", last_msg.ubx, last_msg.cls, last_msg.id, last_msg.length);
            for (auto byte : last_msg.payload) {
                Serial.printf("%02x ", byte);
            }
            Serial.printf("| %02x %02x\n", last_msg.cka, last_msg.ckb);
        }

        //string tmp = gps_manager.read();
        //if (tmp.length()) {
        //    Serial.printf("%s\n", tmp.c_str());
        //}

        if (millis() - timer > 2000 && msg_ubx_q.size()) {
            timer = millis();
            if (msg_ubx_q.size()) {
                auto msg = msg_ubx_q.front();
                for (auto item : get<2>(msg))
                    printf("%d\n", item);
                gps_manager.write(get<0>(msg), get<1>(msg), get<2>(msg));
                msg_ubx_q.pop_front();
            }
        }
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
            if (!in.length() || c == '\r')
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


            try {
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

                else if (cmd == "MOVE_YAW_BY") {
                    moveYawBy(deg2step(std::stoi(arg), YAW_RATIO));
                }
                else if (cmd == "MOVE_PITCH_BY") {
                    movePitchBy(deg2step(std::stoi(arg), PITCH_RATIO));
                }
                else if (cmd == "MOVE_YAW_TO") {
                    moveYawTo(deg2step(std::stoi(arg), YAW_RATIO));
                }
                else if (cmd == "MOVE_PITCH_TO") {
                    movePitchTo(deg2step(std::stoi(arg), PITCH_RATIO));
                }
                else if (cmd == "HOME") {
                    homePitch();
                    homeYaw();
                }
                else if (cmd == "HOME_YAW") {
                    homeYaw();
                }
                else if (cmd == "HOME_PITCH") {
                    homePitch();
                }
                else if (cmd == "DISABLE_STEPPERS") {
                    stepper_pitch.disableOutputs();
                    stepper_yaw.disableOutputs();
                    pitch_homed = false;
                    yaw_homed = false;
                }
                else {
                    printf("UNKNOWN COMMAND\n");
                }
            }
            catch (exception ex) {
                printf("Invalid argument\n");
            }

            in.clear();
        }
    }
}




void setup() {
    // put your setup code here, to run once:
    pinMode(ENDSTOP_PITCH_MIN, INPUT_PULLUP);
    pinMode(ENDSTOP_YAW_MIN, INPUT_PULLUP);
    pinMode(ENDSTOP_YAW_MAX, INPUT_PULLUP);
    
    delay(3000);
    Serial.begin(115200);

    //xTaskCreate()
    xTaskCreate(gpsTask, "GPS", 2048, nullptr, 0, &gps_task);
    xTaskCreate(commsTask, "COMMS", 1024, nullptr, 0, &comms_task);

    stepper_yaw.setMaxSpeed(1000);
    stepper_pitch.setMaxSpeed(1000);

    //xTaskCreate(stepperTask, "STEPPER", 2048, nullptr, 0, &stepper_task);
    //vTaskDelete(nullptr);
}



void loop() {
    //vTaskDelay(1);
    if (pitch_homed)
        stepper_pitch.runSpeedToPosition();
    if (yaw_homed)
        stepper_yaw.runSpeedToPosition();

}
