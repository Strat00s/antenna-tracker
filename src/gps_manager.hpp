#include <Arduino.h>
#include <vector>
#include <stdint.h>

using namespace std;

//TODO wait for response on writes

typedef struct{
    bool ubx;
    uint8_t cls;
    uint8_t id;
    uint16_t length;
    deque<uint8_t> payload;
    uint8_t cka;
    uint8_t ckb;
} gps_msg_t;



class GPSManager {
private:
    HardwareSerial *serial = nullptr;
    gps_msg_t last_msg;

    typedef enum {
        NONE = 0,
        NMEA_START,
        NMEA_PAYLOAD,
        NMEA_END,
        UBX_SYNC1,
        UBX_SYNC2,
        UBX_CLASS,
        UBX_ID,
        UBX_LEN1,
        UBX_LEN2,
        UBX_PAYLOAD_END,
        UBX_CKA,
	} states;

public:
    GPSManager() = default;
    ~GPSManager() = default;

    /** @brief Configure GPS baud rate*/
    int begin(HardwareSerial *serial, unsigned long baud = 9600) {
        this->serial = serial;
        serial->begin(9600);
        return setBaudRate(baud);
    }


    uint8_t setBaudRate(uint32_t baud) {
        //TODO read config first and modify only baud
        //set baudrate, disable NMEA on uart
        write(0x06, 0x00, {1, 0, 0, 0, 0xC0, 0x08, 0, 0, uint8_t(baud), uint8_t(baud >> 8), uint8_t(baud >> 16), uint8_t(baud >> 24), 1, 0, 1, 0, 0, 0, 0, 0});
        serial->flush();
        serial->end();
        serial->begin(baud);
        return getResponse();
    }

    uint8_t getVersion() {
        write(0x0A, 0x04, {});
        return getResponse(0x0A, 0x04);
    }

    uint8_t configure(uint8_t id, vector<uint8_t> payload) {
        write(0x06, id, payload);
        return getResponse();
    }


    const gps_msg_t &getLastMsg() {
        return last_msg;
    }


    /** @brief Wait for specific response from the GPS.
     * 
     * @param cls Response class
     * @param id Response ID
     * @return 0 on success.
     * 1 on failure to read data from serial.
     * 2 on response timeout.
     */
    uint8_t getResponse(uint16_t cls = 0x05, uint8_t id = 0x01) {
        int32_t ret = 0;
        unsigned long timer = millis();
        while (ret != ((cls << 8) | id)) {
            ret = read();
            if (ret < 0)
                return 1;
            if (millis() - timer >= 1100)
                return 2;
        }
        return 0;
    }

    /** @brief Send UBX message to GPS.
     * 
     * @param cls Message class.
     * @param msg_id Message ID.
     * @param payload Payload.
     */
    void write(uint8_t cls, uint8_t msg_id, vector<uint8_t> payload) {
        size_t msg_len = 6 + payload.size() + 2;
        vector<uint8_t> msg;
        msg.reserve(msg_len);
        msg.push_back(0xB5);
        msg.push_back(0x62);
        msg.push_back(cls);
        msg.push_back(msg_id);
        msg.push_back(payload.size());
        msg.push_back(payload.size() >> 8);

        for (auto i : payload)
            msg.push_back(i);

        msg.push_back(0);
        msg.push_back(0);
        for (size_t i = 2; i < msg.size() - 2; i++) {
            msg[msg_len - 2] = msg[msg_len - 2] + msg[i];
            msg[msg_len - 1] = msg[msg_len - 1] + msg[msg_len - 2];
        }

        serial->write(msg.data(), msg.size());
    }


    /** @brief Construct message from GPS serial data.
     * Has to be called repeatedly to construct the message.
     * Returns class and ID of constructed message (UBX) or 0xFFFF (NMEA).
     * 
     * @return 0 when nothing or when processing new message.
     * class | ID when message was processed succesfully (0xFFFF for NMEA).
     * -1 on processing timeout (took longer than 0.5s).
     */
    int32_t read() {
        static int state = 0;
        static uint16_t len = 0;
        static unsigned long timer = 0;


        //timeout
        if (millis() - timer >= 500 && state != NONE && serial->available() <= 0) {
            state = NONE;
            return -1;
        }


        int c = serial->read();
        if (c < 0)
            return 0;

        if (state == NONE && c == '$') {
            timer = millis();
            state = NMEA_START;
            last_msg.ubx    = false;
            last_msg.cls    = 0xFF;
            last_msg.id     = 0xFF;
            last_msg.length = 0;
            last_msg.payload.clear();
        }
        else if (state == NMEA_START) {
            if (c == '\r')
                state = NMEA_END;
            else {
                last_msg.length++;
                last_msg.payload.push_back(c);
            }
        }
        else if (state == NMEA_END && c == '\n') {
            state = NONE;
            return uint16_t(last_msg.cls) << 8 | last_msg.id;
        }

        else if (state == NONE && c == 0xB5) {
            timer = millis();
            state = UBX_SYNC1;
            last_msg.ubx    = true;
            last_msg.cls    = 0;
            last_msg.id     = 0;
            last_msg.length = 0;
            last_msg.payload.clear();
            len = 0;
        }
        else if (state == UBX_SYNC1 && c == 0x62) {
            state = UBX_SYNC2;
        }
        else if (state == UBX_SYNC2) {
            state = UBX_CLASS;
            last_msg.cls = c;
        }
        else if (state == UBX_CLASS) {
            state = UBX_ID;
            last_msg.id = c;
        }
        else if (state == UBX_ID) {
            state = UBX_LEN1;
            last_msg.length = uint16_t(c);
        }
        else if (state == UBX_LEN1) {
            state = UBX_LEN2;
            last_msg.length |= uint16_t(c) << 8;
            len = last_msg.length;
        }
        else if (state == UBX_LEN2 && len) {
            len--;
            last_msg.payload.push_back(c);
            if (!len)
                state = UBX_PAYLOAD_END;
        }
        else if (state == UBX_PAYLOAD_END) {
            state = UBX_CKA;
            last_msg.cka = c;
        }
        else if (state == UBX_CKA) {
            state = NONE;
            last_msg.ckb = c;
            return uint16_t(last_msg.cls) << 8 | last_msg.id;
        }

        return 0;
    }


};