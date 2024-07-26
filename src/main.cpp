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



StepperManager steppers(PITCH_N1, PITCH_N2, PITCH_N3, PITCH_N4, YAW_N1, YAW_N2, YAW_N3, YAW_N4, ENDSTOP_PITCH_MIN, ENDSTOP_YAW_MIN, ENDSTOP_YAW_MAX);


double home_lat  = 0;
double home_lon  = 0;
int32_t home_alt = 0;

double target_lat  = 0;
double target_lon  = 0;
int32_t target_alt = 0;


GPSManager gps_manager;



// Function to convert degrees to radians
double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

/** @brief Calculate bearing from first location to second with respect to north
 * 
 * @param lat1 
 * @param lon1 
 * @param lat2 
 * @param lon2 
 * @return double
 */
double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    //convert latitude and longitude from degrees to radians
    double phi1 = degToRad(lat1);
    double phi2 = degToRad(lat2);
    double deltaLambda = degToRad(lon2 - lon1);

    //calculate the bearing
    double y = std::sin(deltaLambda) * std::cos(phi2);
    double x = std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(deltaLambda);
    double theta = std::atan2(y, x);

    //convert the bearing from radians to degrees
    double thetaDegrees = theta * 180.0 / M_PI;

    //cormalize the angle to be within the range 0° to 360°
    double bearing = std::fmod((thetaDegrees), 360.0);

    return bearing;
}

// Function to calculate distance using the Haversine formula
/** @brief Calculate distance between 2 locations
 * 
 * @param lat1 
 * @param lon1 
 * @param lat2 
 * @param lon2 
 * @return double 
 */
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    //convert latitude and longitude from degrees to radians
    double phi1 = degToRad(lat1);
    double phi2 = degToRad(lat2);
    double deltaPhi = degToRad(lat2 - lat1);
    double deltaLambda = degToRad(lon2 - lon1);

    //haversine formula
    double a = std::sin(deltaPhi / 2.0) * std::sin(deltaPhi / 2.0) +
               std::cos(phi1) * std::cos(phi2) * 
               std::sin(deltaLambda / 2.0) * std::sin(deltaLambda / 2.0);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    //distance in kilometers
    return c * 6371.0 * 1000; //distance in meters
}


/** @brief Calculate angle between 2 elevations.
 * 
 * @param height1
 * @param height2
 * @param distance Distance in meters.
 * @return double 
 */
double calculatePitchAngle(int32_t height1, int32_t height2, double distance) {
    return std::atan2(height2 - height1, distance) * 180.0 / M_PI; //back to degrees
}


void handleGps() {
    auto ret = gps_manager.read();
    if (!ret)
        return;
    if (ret < 0) {
        printf("ERROR in read: %d\n", ret);
        return;
    }


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
        return;
    }

    //PVT
    if (last_msg.cls == 0x01 && last_msg.id == 0x07) {
        //date time
        uint16_t year = uint16_t(last_msg.payload[5]) << 8 | last_msg.payload[4];
        uint8_t month = last_msg.payload[6];
        uint8_t day   = last_msg.payload[7];
        uint8_t hour  = last_msg.payload[8] + UTC_OFFSET;
        uint8_t min   = last_msg.payload[9];
        uint8_t sec   = last_msg.payload[10];
        uint32_t tacc = uint32_t(last_msg.payload[15]) << 24 | uint32_t(last_msg.payload[14]) << 16 | uint16_t(last_msg.payload[13]) << 8 | last_msg.payload[12];

        uint8_t valid    = last_msg.payload[11]; //valid time, date, mag, fully resolved, 
        uint8_t fix_type = last_msg.payload[20]; //2d, 3d, ...
        uint8_t flags    = last_msg.payload[21]; //gnss fix ok
        uint8_t sats     = last_msg.payload[23]; //number of satelites
        uint8_t flags3   = last_msg.payload[79]; //invalid lat, lon, height

        home_lat         = (uint32_t(last_msg.payload[31]) << 24 | uint32_t(last_msg.payload[30]) << 16 | uint16_t(last_msg.payload[29]) << 8 | last_msg.payload[28]) * 1e-7;
        home_lon         = (uint32_t(last_msg.payload[27]) << 24 | uint32_t(last_msg.payload[26]) << 16 | uint16_t(last_msg.payload[25]) << 8 | last_msg.payload[24]) * 1e-7;
        home_alt         = (uint32_t(last_msg.payload[39]) << 24 | uint32_t(last_msg.payload[38]) << 16 | uint16_t(last_msg.payload[37]) << 8 | last_msg.payload[36]) / 1000;
    }

    //SAT
    else if (last_msg.cls == 0x01 && last_msg.id == 0x35) {
        printf("Sat info\n");
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


void handleComms() {
    static string in = "";
    if (Serial.available() <= 0)
        return;

    int c = Serial.read();
    if (c < 0) {
        printf("\nFailed to read input!\n");
        return;
    }
    if (c != '\n' && c != '\r') {
        Serial.print(char(c));
        in += c;
        return;
    }
    if (!in.length() || c == '\r')
        return;

    printf("\n");
    size_t split = in.find_first_of(' ');
    string cmd = in;
    string arg = "";

    //no argument
    if (split == string::npos) {
        if (cmd == "CALIBRATE") {
            steppers.calibrate();
        }
        else if (cmd == "HOME") {
            steppers.home();
        }
        else if (cmd == "DISABLE") {
            steppers.disable();
        }
        in.clear();
        return;
    }

    cmd = in.substr(0, split);
    arg = in.substr(split + 1);

    try {
        if (cmd == "TARGET_LAT") {
            target_lat = std::stod(arg.substr(0, split));
            printf("NOT YET TESTED\n");
        }
        else if (cmd == "TARGET_LON") {
            target_lon = std::stod(arg.substr(0, split));
            printf("NOT YET TESTED\n");
        }
        else if (cmd == "TARGET_ALT") {
            target_alt = std::stoi(arg.substr(0, split));
            printf("NOT YET TESTED\n");
        }

        else if (cmd == "ARMED") {
            printf("NOT YET IMPLEMENTED\n");
        }

        else if (cmd == "HOME_LAT") {
            //if not gps || gps lock
            printf("NOT YET IMPLEMENTED\n");
        }
        else if (cmd == "HOME_LON") {
            //if not gps || gps lock
            printf("NOT YET IMPLEMENTED\n");
        }
        else if (cmd == "HOME_ALT") {
            //if not gps || gps lock
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
        else if (cmd == "HOME_YAW") {
            steppers.home(false, true);
        }
        else if (cmd == "HOME_PITCH") {
            steppers.home(true, false);
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





void setup() {
    // put your setup code here, to run once:
    pinMode(ENDSTOP_PITCH_MIN, INPUT_PULLUP);
    pinMode(ENDSTOP_YAW_MIN, INPUT_PULLUP);
    pinMode(ENDSTOP_YAW_MAX, INPUT_PULLUP);
    
    delay(3000);
    Serial.begin(115200);


    GPS_SERIAL.setTX(GPS_RX);
    GPS_SERIAL.setRX(GPS_TX);
    GPS_SERIAL.setFIFOSize(512);
    int ret = gps_manager.begin(&GPS_SERIAL, 115200);
    if (ret && gps_manager.getVersion()) {
        printf("Failed to change baud rate: %d\n", ret);
        vTaskDelete(nullptr);
    }
    ret = gps_manager.configure(0x01, {0x01, 0x07, 1});
    if (ret) {
        printf("Failed to change config: %d\n", ret);
        printf("FIX!\n");
        vTaskDelete(nullptr);
    }
    ret = gps_manager.configure(0x01, {0x01, 0x35, 1});
    if (ret) {
        printf("Failed to change config: %d\n", ret);
        printf("FIX!\n");
        vTaskDelete(nullptr);
    }


    steppers.begin(1000, 1000, 500, 500, PITCH_RATIO, YAW_RATIO);
    steppers.calibrate();
}


double old_pitch_angle = -99999;
double old_yaw_angle   = -99999;

void loop() {
    handleComms();
    handleGps();
    if (steppers.isCalibrated()) {
        auto distance = calculateDistance(home_lat, home_lon, target_lat, target_lon);
        auto pitch_angle = calculatePitchAngle(home_alt, target_alt, 1000.0);
        auto yaw_angle = calculateBearing(home_lat, home_lon, target_lat, target_lon);

        if (abs(pitch_angle - old_pitch_angle) >= 5 || abs(yaw_angle - old_yaw_angle) >= 5) {
            steppers.moveTo(pitch_angle, yaw_angle);
            old_pitch_angle = pitch_angle;
            old_yaw_angle = yaw_angle;
            printf("HOME:\n");
            printf("  LAT: %lf\n", home_lat);
            printf("  LON: %lf\n", home_lon);
            printf("  ALT: %ld\n", home_alt);
            printf("TARGET:\n");
            printf("  LAT: %lf\n", target_lat);
            printf("  LON: %lf\n", target_lon);
            printf("  ALT: %ld\n", target_alt);
            printf("RESULT:\n");
            printf("  PITCH: %lf\n", pitch_angle);
            printf("  YAW:   %lf\n", yaw_angle);
        }
    }
    steppers.run();
}
