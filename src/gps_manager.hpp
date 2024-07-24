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

    uint8_t getResponse() {
        unsigned long timer = millis();
        int32_t ret = 0;
        while (ret >> 8 != 0x05) {
            ret = read();
            if (ret > 0)
                timer = millis();
            if (ret == -1)
                return 2;
            if (millis() - timer >= 1000)
                return 3;
        }
        return uint8_t(ret) ? 0 : 1;
    }

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
        write(0x06, 0x00, {1, 0, 0, 0, 0xC0, 0x08, 0, 0, uint8_t(baud), uint8_t(baud >> 8), uint8_t(baud >> 16), uint8_t(baud >> 24), 3, 0, 1, 0, 0, 0, 0, 0});
        serial->flush();
        serial->end();
        serial->begin(baud);
        return getResponse();
    }

    uint8_t configure(uint8_t id, vector<uint8_t> payload) {
        write(0x06, id, payload);
        return getResponse();
    }

    uint8_t getVersion() {
        write(0x0A, 0x04, {});
        unsigned long timer = millis();
        int32_t ret = 0;
        while (ret >> 8 != 0x05) {
            ret = read();
            if (ret > 0)
                break;
            if (ret == -1)
                return 2;
            if (millis() - timer >= 1000)
                return 3;
        }
        return 0;
    }


    const gps_msg_t &getLastMsg() {
        return last_msg;
    }


    /** @brief UBX write.
     * 
     * @param cls Message class.
     * @param msg_id Message ID.
     * @param payload Payload.
     */
    void write(uint8_t cls, uint8_t msg_id, vector<uint8_t> payload) {
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

        msg[msg_len - 2] = 0;
        msg[msg_len - 1] = 0;
        for (size_t i = 2; i < msg.size() - 2; i++) {
            msg[msg_len - 2] = msg[msg_len - 2] + msg[i];
            msg[msg_len - 1] = msg[msg_len - 1] + msg[msg_len - 2];
        }

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
        serial->printf("$%s*%02x\r\n", msg, checksum);
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
        if (serial->available() > 0) {
            int c = serial->read();
            if (c < 0)
                return 0;
            
            //NMEA start
            if (c == '$' && state == 0) {
                state = 1;
                timer = millis();
                last_msg.ubx = false;
                last_msg.cls = 0xFF;
                last_msg.id = 0xFF;
                last_msg.length = 0;
                last_msg.payload.clear();
                return 0;
            }
            //NMEA data
            else if (state == 1) {
                if (c == '\r') {
                    //printf("\\R\n");
                    state = 2;
                }
                else {
                    last_msg.length++;
                    last_msg.payload.push_back(c);
                }
                return 0;
            }
            //NMEA end
            else if (c == '\n' && state == 2) {
                state = 0;
                return uint16_t(last_msg.cls) << 8 | last_msg.id;
            }

            //UBX sync 1
            else if (c == 0xB5 && state == 0) {
                state = 3;
                timer = millis();
                last_msg.ubx = true;
                last_msg.cls = 0;
                last_msg.id = 0;
                last_msg.length = 0;
                last_msg.payload.clear();
                len = 0;
                return 0;
            }

            //UBX sync 2
            else if (c == 0x62 && state == 3) {
                state = 4;
                return 0;
            }
            //UBX class
            else if (state == 4) {
                state = 5;
                last_msg.cls = c;
                return 0;
            }
            //UBX ID
            else if (state == 5) {
                state = 6;
                last_msg.id = c;
                return 0;
            }
            //UBX len
            else if (state == 6) {
                state = 7;
                last_msg.length = uint16_t(c);
                return 0;
            }
            //UBX len
            else if (state == 7) {
                state = 8;
                last_msg.length |= uint16_t(c) << 8;
                len = last_msg.length;
                //printf("%d\n", len);
                return 0;
            }
            //UBX payload
            else if (len && state == 8) {
                last_msg.payload.push_back(c);
                len--;
                if (!len)
                    state = 9;
                return 0;
            }
            //UBX CKA
            else if (state == 9) {
                state = 10;
                last_msg.cka = c;
                return 0;
            }
            //UBX CKB
            else if (state == 10) {
                state = 0;
                last_msg.ckb = c;
                return uint16_t(last_msg.cls) << 8 | last_msg.id;
            }
        }
        
        //timeout
        if (millis() - timer >= 500 && state != 0) {
            state = 0;
            return -1;
        }
        return 0;
    }
};