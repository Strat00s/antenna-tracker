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


typedef struct{
    bool ubx;
    uint8_t cls;
    uint8_t id;
    uint16_t length;
    deque<uint8_t> payload;
    uint8_t cka;
    uint8_t ckb;
} ubx_msg_t;


class GPSManager {
private:
    HardwareSerial *serial = nullptr;

    string int2hex(uint8_t x) {
        char buf[4] = {0};
        sprintf(buf, "%02x ", x);
        return buf;
    }

public:
    GPSManager() = default;
    ~GPSManager() = default;

    ubx_msg_t last_msg;

    /** @brief Configure GPS baud rate*/
    void begin(HardwareSerial *serial, unsigned long baud = 9600) {
        this->serial = serial;
        serial->begin(9600);
        setBaudRate(baud);
    }

    void setBaudRate(uint32_t baud) {
        //TODO read config first and modify only baud
        //set baudrate, disable NMEA on uart
        write(0x06, 0x00,{1, 0, 0, 0, 0xC0, 0x08, 0, 0, uint8_t(baud), uint8_t(baud >> 8), uint8_t(baud >> 16), uint8_t(baud >> 24), 3, 0, 1, 0, 0, 0, 0, 0});
        //TODO wait for response
        vTaskDelay(100);
        serial->end();
        serial->begin(baud);
    }

    /** @brief UBX write.
     * 
     * @param cls Message class.
     * @param msg_id Message ID.
     * @param payload Payload.
     */
    void write(uint8_t cls, uint8_t msg_id, vector<uint8_t> payload) {
        //calculate checksum
        uint16_t checksum = 0;
        size_t msg_len = 6 + payload.size() + 2;
        vector<uint8_t> msg;
        msg.resize(msg_len);
        msg[0] = 0xB5;
        msg[1] = 0x62;
        msg[2] = cls;
        msg[3] = msg_id;
        msg[4] = payload.size();
        msg[5] = payload.size() >> 8;

        for (size_t i = 0; i < payload.size(); i++) {
            msg[i + 6] = payload[i];
        }

        uint8_t CKA = 0;
        uint8_t CKB = 0;
        for (size_t i = 2; i < msg.size() - 2; i++) {
            CKA = CKA + msg[i];
            CKB = CKB + CKA;
        }
        msg[msg_len - 2] = CKA;
        msg[msg_len - 1] = CKB;
        printf("UBX payload to sent (%d): ", msg.size());
        for (size_t i = 0; i < msg.size(); i++) {
            printf("%0x ", msg[i]);
        }
        printf("\n");
        serial->write(msg.data(), msg.size());
    }

    /** @brief NMEA write.
     * 
     * @param msg Message to send.
     */
    void write(const char *msg) {
        //calculate checksum
        uint8_t checksum = 0;
        for (const char *c = msg; *c != '\0'; c++)
            checksum ^= *c;
        printf("$%s*%02x\r\n", msg, checksum);
        serial->printf("$%s*%02x\r\n", msg, checksum);
    }

    
    uint16_t read() {
        static int state = 0;
        static uint16_t len = 0;
        if (serial->available() > 0) {
            int c = serial->read();
            if (c < 0)
                return 0;
            
            if (c == '$' && state == 0) {
                state = 1;
                last_msg.ubx = false;
                last_msg.cls = 0xFF;
                last_msg.id = 0xFF;
                last_msg.length = 0;
                last_msg.payload.clear();
                return 0;
            }
            else if (state == 1) {
                if (c == '\r')
                    state = 2;
                else {
                    last_msg.length++;
                    last_msg.payload.push_back(c);
                }
                return 0;
            }
            else if (c == '\n' && state == 2) {
                state = 0;
                return uint16_t(last_msg.cls) << 8 | last_msg.id;
            }

            else if (c == 0xB5 && state == 0) {
                printf("SYNC\n");
                state = 3;
                last_msg.ubx = true;
                last_msg.cls = 0;
                last_msg.id = 0;
                last_msg.length = 0;
                last_msg.payload.clear();
                len = 0;
                return 0;
            }
            else if (c == 0x62 && state == 3) {
                printf("SYNC\n");
                state = 4;
                return 0;
            }
            else if (state == 4) {
                printf("CLASS\n");

                state = 5;
                last_msg.cls = c;
                return 0;
            }
            else if (state == 5) {
                printf("ID\n");

                state = 6;
                last_msg.id = c;
                return 0;
            }
            else if (state == 6) {
                printf("LEN\n");

                state = 7;
                last_msg.length = uint16_t(c);
                return 0;
            }
            else if (state == 7) {
                printf("LEN\n");

                state = 8;
                last_msg.length |= uint16_t(c) << 8;
                len = last_msg.length;
                printf("%d\n", len);
                return 0;
            }
            else if (len && state == 8) {
                printf("PAYLOAD %d\n", len);

                last_msg.payload.push_back(c);
                len--;
                if (!len)
                    state = 9;
                return 0;
            }
            else if (state == 9) {
                printf("CKA\n");

                state = 10;
                last_msg.cka = c;
                return 0;
            }
            else if (state == 10) {
                printf("CKB\n");

                state = 0;
                last_msg.ckb = c;
                return uint16_t(last_msg.cls) << 8 | last_msg.id;
            }
        }
        return 0;
    }

    /*
    string read() {
        static string msg = "";
        static int state = 0;
        static uint16_t msg_len = 0;
        if (serial->available() > 0) {
            int c = serial->read();
            if (c < 0)
                return "";
            
            if (c == '$' && state == 0) {
                state = 1;
                msg.clear();
                msg += c;
                return "";
            }
            else if (c == '\r' && state == 1) {
                state = 2;
                return "";
            }            
            else if (c == '\n' && state == 2) {
                state = 0;
                return msg;
            }

            else if (c == 0xB5 && state == 0) {
                msg.clear();
                msg += int2hex(c);
                state = 3;
                return "";
            }
            else if (c == 0x62 && state == 3) {
                state = 4;
                msg += int2hex(c);
            }
            else if (state == 4 || state == 5) {
                state++;
                msg += int2hex(c);
                return "";
            }
            else if (state == 6) {
                msg_len = uint16_t(c);
                msg += int2hex(c);
                state = 7;
                return "";
            }
            else if (state == 7) {
                msg_len |= uint16_t(c) << 8;
                msg += int2hex(c);
                state = 8;
                return "";
            }
            else if (msg_len && state == 8) {
                msg_len--;
                msg += int2hex(c);
                if (!msg_len)
                    state = 9;
                return "";
            }
            else if (state == 9) {
                state = 10;
                msg += int2hex(c);
                return "";
            }
            else if (state == 10) {
                state = 0;
                msg += int2hex(c);
                return msg;
            }

            msg += c;
        }
        return "";
    }
    */
};


GPSManager gps_manager;


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

void movePitchTo(int pos, int speed = 500) {
    stepper_pitch.moveTo(pos * pitch_dir);
    movePitch(speed);
}

void moveYawTo(int pos, int speed = 500) {
    stepper_yaw.moveTo(pos * yaw_dir);
    moveYaw(speed);
}

void movePitchBy(int pos, int speed = 500) {
    stepper_pitch.move(pos * pitch_dir);
    movePitch(speed);
}

void moveYawBy(int pos, int speed = 500) {
    stepper_yaw.move(pos * yaw_dir);
        moveYaw(speed);

}

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



void gpsTask(void *) {
    GPS_SERIAL.setTX(GPS_RX);
    GPS_SERIAL.setRX(GPS_TX);
    GPS_SERIAL.setFIFOSize(256);
    gps_manager.begin(&GPS_SERIAL, 115200);


    deque<tuple<uint8_t, uint8_t, vector<uint8_t>>> msg_ubx_q;
    msg_ubx_q.push_back({0x06, 0x17,{0, 0x4b, 0, 0b00001000, 0, 0, 0, 0, 1, 0, 0, 0x01, 0, 0, 0, 0, 0, 0, 0, 0}});
    msg_ubx_q.push_back({0x06, 0x3E, {}});
    //msg_ubx_q.push_back({0x06, 0x02, {0, 0, 0, 0, 0b00011111, 1, 0, 0, 0, 0}});
    
    unsigned long timer = millis();

    while (true) {
        if (gps_manager.read()) {
            Serial.printf("%d %02x %02x %d | ", gps_manager.last_msg.ubx, gps_manager.last_msg.cls, gps_manager.last_msg.id, gps_manager.last_msg.length);
            for (auto byte : gps_manager.last_msg.payload) {
                Serial.printf("%02x ", byte);
            }
            Serial.printf("| %02x %02x\n", gps_manager.last_msg.cka, gps_manager.last_msg.ckb);
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

    xTaskCreate(gpsTask, "GPS", 2048, nullptr, 0, &gps_task);
    xTaskCreate(commsTask, "COMMS", 1024, nullptr, 0, &comms_task);

    stepper_yaw.setMaxSpeed(1000);
    stepper_pitch.setMaxSpeed(1000);

    //xTaskCreate(stepperTask, "STEPPER", 2048, nullptr, 0, &stepper_task);
    //vTaskDelete(nullptr);
}



void loop() {
    //vTaskDelay(1);
}
