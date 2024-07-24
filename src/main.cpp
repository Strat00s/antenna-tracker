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
#include "stepper_manager.hpp"

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

#define PITCH_RATIO 50.0 / 14.0
//#define YAW_RATIO 60 / 20
#define YAW_RATIO 60.0 / 25.0

#define UTC_OFFSET 2

//#define deg2step(deg, ratio) (deg * 32 * 63.68395 / 360 * ratio)


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

//AccelStepper stepper_pitch(AccelStepper::FULL4WIRE, PITCH_N1, PITCH_N3, PITCH_N2, PITCH_N4);
//AccelStepper stepper_yaw(AccelStepper::FULL4WIRE, YAW_N1, YAW_N3, YAW_N2, YAW_N4);
//MultiStepper steppers;
StepperManager steppers(PITCH_N1, PITCH_N2, PITCH_N3, PITCH_N4, YAW_N1, YAW_N2, YAW_N3, YAW_N4, ENDSTOP_PITCH_MIN, ENDSTOP_YAW_MIN, ENDSTOP_YAW_MAX);


int yaw_dir = 1;
int pitch_dir = 1;

bool yaw_calibrated = false;
bool pitch_calibrated = false;

bool yaw_homed = false;
bool pitch_homed = false;

long pitch_steps = 999999;
long yaw_steps   = 999999;



GPSManager gps_manager;


void stepperTask(void *) {
    while (true) {
        steppers.run();
    }
}

void gpsTask(void *) {
    GPS_SERIAL.setTX(GPS_RX);
    GPS_SERIAL.setRX(GPS_TX);
    GPS_SERIAL.setFIFOSize(256);
    int ret = gps_manager.begin(&GPS_SERIAL, 115200);
    if (ret && gps_manager.getVersion()) {
        printf("Failed to change baud rate: %d\n", ret);
        vTaskDelete(nullptr);
    }


    //protocol config
    ret = gps_manager.configure(0x02, {0, 0, 0, 0, 0, 0b00011111, 0, 0, 0, 0});
    if (ret) {
        printf("Failed to change config: %d\n", ret);
        printf("FIX!\n");
        vTaskDelete(nullptr);
    }
    //periodic PVT reports
    ret = gps_manager.configure(0x01, {0x01, 0x07, 0, 1, 0, 0, 0, 0});
    if (ret) {
        printf("Failed to change config: %d\n", ret);
        printf("FIX!\n");
        vTaskDelete(nullptr);
    }

    //handle incoming messages
    while (true) {
        auto ret = gps_manager.read();
        if (ret > 0) {
            auto last_msg = gps_manager.getLastMsg();
            //checksum
            uint8_t cka = 0;
            uint8_t ckb = 0;
            cka += last_msg.cls;
            ckb += cka;
            cka += last_msg.id;
            ckb += cka;
            cka += last_msg.length;
            ckb += cka;
            cka += last_msg.length >> 8;
            ckb += cka;
            for (auto i : last_msg.payload) {
                cka += i;
                ckb += cka;
            }
            if (cka != last_msg.cka || ckb != last_msg.ckb) {
                printf("Checksum missmatch: %d - %d | %d - %d\n", cka, last_msg.cka, ckb, last_msg.ckb);
                continue;
            }
            //class and id
            if (last_msg.cls == 0x01 && last_msg.id == 0x07) {
                //handle it
                uint32_t itow = uint32_t(last_msg.payload[3]) << 24 | uint32_t(last_msg.payload[2]) << 16 | uint16_t(last_msg.payload[1]) << 8 | last_msg.payload[0];

                uint16_t year = uint16_t(last_msg.payload[5]) << 8 | last_msg.payload[4];
                uint8_t month = last_msg.payload[6];
                uint8_t day   = last_msg.payload[7];
                uint8_t hour  = last_msg.payload[8] + UTC_OFFSET;
                uint8_t min   = last_msg.payload[9];
                uint8_t sec   = last_msg.payload[10];

                uint8_t valid = last_msg.payload[11];

                uint32_t tacc = uint32_t(last_msg.payload[15]) << 24 | uint32_t(last_msg.payload[14]) << 16 | uint16_t(last_msg.payload[13]) << 8 | last_msg.payload[12];
                int32_t nano = uint32_t(last_msg.payload[19]) << 24 | uint32_t(last_msg.payload[18]) << 16 | uint16_t(last_msg.payload[17]) << 8 | last_msg.payload[16];

                uint8_t fix_type = last_msg.payload[20];
                uint8_t flags    = last_msg.payload[21];
                uint8_t flags2   = last_msg.payload[22];
                uint8_t sats     = last_msg.payload[23];

                printf("%02u.%02u.%04u %02u:%02u:%02u.%08lld\n", day, month, year, hour, min, sec, nano);
                printf("accuracy:   %ldns\n", tacc);
                printf("valid time: %u\n", valid & 0b00000001);
                printf("valid date: %u\n", valid >> 1 & 0b00000001);
                printf("resolved:   %u\n", valid >> 2 & 0b00000001);
                printf("valid mag:  %u\n\n", valid >> 3 & 0b00000001);
                printf("sattelites: %u\n", sats);
                printf("fix type:   %u\n", fix_type);
            }
            else {
                printf("other message: \n");
                Serial.printf("%d %02x %02x %d | ", last_msg.ubx, last_msg.cls, last_msg.id, last_msg.length);
                for (auto byte : last_msg.payload) {
                    Serial.printf("%02x ", byte);
                }
                Serial.printf("| %02x %02x\n", last_msg.cka, last_msg.ckb);
            }
        }
        else if (ret < 0) {
            printf("ERROR in read: %d\n", ret);
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
            size_t split = in.find_first_of(' ');
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

                else if (cmd == "MOVE_BY") {
                    split = arg.find_first_of(' ');
                    if (split != string::npos)
                        steppers.moveBy(std::stoi(arg.substr(0, split)), std::stoi(arg.substr(split + 1)));
                    else
                        printf("invalid argument count\n");
                }
                else if (cmd == "MOVE_TO") {
                    split = arg.find_first_of(' ');
                    if (split != string::npos)
                        steppers.moveTo(std::stoi(arg.substr(0, split)), std::stoi(arg.substr(split + 1)));
                    else
                        printf("invalid argument count\n");
                }
                else if (cmd == "CALIBRATE") {
                    steppers.calibrate();
                }
                else if (cmd == "HOME") {
                    steppers.home();
                }
                else if (cmd == "HOME_YAW") {
                    steppers.home(false, true);
                }
                else if (cmd == "HOME_PITCH") {
                    steppers.home(true, false);
                }
                else if (cmd == "DISABLE") {
                    steppers.disable();
                }
                else {
                    printf("UNKNOWN COMMAND\n");
                }
            }
            catch (exception ex) {
                printf("Exception occured: %s\n", ex.what());
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

    steppers.begin(1000, 1000, 500, 500, PITCH_RATIO, YAW_RATIO);

    //xTaskCreate()
    xTaskCreate(gpsTask, "GPS", 2048, nullptr, 0, &gps_task);
    xTaskCreate(commsTask, "COMMS", 1024, nullptr, 0, &comms_task);

    //stepper_yaw.setMaxSpeed(1000);
    //stepper_pitch.setMaxSpeed(1000);

    //xTaskCreate(stepperTask, "STEPPER", 2048, nullptr, 0, &stepper_task);
    //vTaskDelete(nullptr);
}



void loop() {
    steppers.run();
}
