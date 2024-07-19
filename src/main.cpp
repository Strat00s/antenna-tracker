#include <Arduino.h>
#include <I2C.h>
#include <SPI.h>
#include <Serial.h>


/*---(PIN DEFINITIONS)---*/
//endstops
#define ENDSTOP_PITCH
#define ENDSTOP_ROT1
#define EDNSTOP_ROT2
//gps
#define GPS_TX
#define GPS_RX
//i2c
#define SDA
#define SCK
//motors
#define M1_N1
#define M1_N2
#define M1_N3
#define M1_N4
#define M2_N1
#define M2_N2
#define M2_N3
#define M2_N4
//buttons
#define FWD
#define BCK
#define OK
//comms
#define COMMS_TX
#define COMMS_RX


//1. Read config
//4. Home
//2. Check if GPS is connected
//3. Check if compass is connected

void setup() {
    // put your setup code here, to run once:
}

void loop() {
    // put your main code here, to run repeatedly:
}
