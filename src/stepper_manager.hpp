#include <Arduino.h>
#include <AccelStepper.h>


//TODO home as single routine

class StepperManager {
private:
    /* data */
    AccelStepper pitch;
    AccelStepper yaw;

    uint8_t pitch_homed = NONE;
    uint8_t yaw_homed = NONE;

    uint8_t calibrated = NONE;

    int32_t pitch_steps = INT32_MAX;
    int32_t yaw_steps = INT32_MAX;

    int pitch_min = -1;
    int yaw_min = -1;
    int yaw_max = -1;


    int8_t pitch_dir = 1;
    int8_t yaw_dir   = -1;

    int pitch_speed = 500;
    int yaw_speed = 500;

    double pitch_ratio = 0;
    double yaw_ratio = 0;


    long deg2step(int deg, double ratio) {
        //return (deg * 32 * 63.68395 / 360) * (ratio);
        printf("Move by: %d %f -> %f %ld\n", deg, ratio, (2048.0 / 360.0) * deg * (ratio), long((2048.0 / 360.0) * deg * (ratio)));
        return (2048.0 / 360.0) * deg * (ratio);
    }

    void move(int pitch_speed, int yaw_speed) {
        pitch.setSpeed(pitch_speed);
        pitch.runSpeedToPosition();
        yaw.setSpeed(yaw_speed);
        yaw.runSpeedToPosition();
    }

public:
    typedef enum {
        NONE = 0,
        STATE1,
        STATE2,
        STATE3,
        STATE4,
        STATE5,
        STATE6,
        DONE
	} states;


    StepperManager(uint8_t pitch1, uint8_t pitch2, uint8_t pitch3, uint8_t pitch4, uint8_t yaw1, uint8_t yaw2, uint8_t yaw3, uint8_t yaw4, int pitch_min, int yaw_min, int yaw_max) {
        yaw = AccelStepper(AccelStepper::FULL4WIRE, yaw1, yaw3, yaw2, yaw4);
        pitch = AccelStepper(AccelStepper::FULL4WIRE, pitch1, pitch3, pitch2, pitch4);
        this->pitch_min = pitch_min;
        this->yaw_max = yaw_max;
        this->yaw_min = yaw_min;
    }
    ~StepperManager();

    void begin(int pitch_max_speed, int yaw_max_speed, int pitch_speed, int yaw_speed, double pitch_ratio, double yaw_ratio) {
        setMaxSpeed(pitch_max_speed, yaw_max_speed);
        setSpeed(pitch_speed, yaw_speed);
        this->pitch_ratio = pitch_ratio;
        this->yaw_ratio = yaw_ratio;
    }

    void setMaxSpeed(int pitch_speed, int yaw_speed) {
        yaw.setMaxSpeed(pitch_speed);
        pitch.setMaxSpeed(yaw_speed);
    }

    void setSpeed(int pitch_speed, int yaw_speed) {
        this->pitch_speed = pitch_speed;
        this->yaw_speed = yaw_speed;
    }

    void disable() {
        pitch.disableOutputs();
        yaw.disableOutputs();
        pitch_homed = NONE;
        yaw_homed = NONE;
    }

    void calibrate() {
        if (calibrated != NONE && calibrated != DONE)
            return;

        calibrated = STATE1;
        pitch_homed = DONE;
        printf("Calibrating PITCH\n");
        pitch.move(deg2step(-180, pitch_ratio) * pitch_dir);
        //yaw
        printf("Calibrating YAW\n");
        //yaw.move(deg2step(360, yaw_ratio) * yaw_dir);
    }

    void home(bool pitch = true, bool yaw = true) {
        printf("WILL HOME\n");
        if (calibrated != DONE) {
            printf("NOT CALIBRATED!\n");
            return;
        }
        if (pitch) {
            pitch_homed = STATE1;
            this->pitch.move(deg2step(-180, pitch_ratio) * pitch_dir);
        }

        if (yaw) {
            yaw_homed = STATE1;
            this->yaw.move(deg2step(360, yaw_ratio) * yaw_dir);
        }
    }


    void moveTo(int16_t pitch_angle, int16_t yaw_angle) {
        if (calibrated != DONE) {
            printf("NOT CALIBRATED!\n");
            return;
        }
        pitch.move(0);
        yaw.move(0);
        pitch.moveTo(deg2step(pitch_angle, pitch_ratio) * pitch_dir);
        yaw.moveTo(deg2step(yaw_angle, yaw_ratio) * yaw_dir);
    }

    void moveBy(int16_t pitch_deg, int16_t yaw_deg) {
        if (calibrated != DONE) {
            printf("NOT CALIBRATED!\n");
            return;
        }
        pitch.move(0);
        yaw.move(0);
        pitch.move(deg2step(pitch_deg, pitch_ratio) * pitch_dir);
        yaw.move(deg2step(yaw_deg, yaw_ratio) * yaw_dir);
    }


    /** @brief Main loop for moving the steppers*/
    void run() {
        static int old_calib = NONE;
        static int old_yaw_dir = yaw_dir;
        static int old_pitch_dir = pitch_dir;
        if (old_calib != calibrated) {
            printf("CALIB: %d\n", calibrated);
            old_calib = calibrated;
        }

        if (old_yaw_dir != yaw_dir) {
            printf("new yaw dir: %d\n", yaw_dir);
            old_yaw_dir = yaw_dir;
        }
        if (old_pitch_dir != pitch_dir) {
            printf("new pitch dir: %d\n", pitch_dir);
            old_pitch_dir = pitch_dir;
        }
        int pitch_togo = pitch.distanceToGo();
        int yaw_togo = yaw.distanceToGo();

        //calibration state machine
        if (calibrated == STATE1) {
            //endstop hit -> move to yaw
            //TODO fix -> if homed, will falsly report
            if (!digitalRead(pitch_min)) {
                pitch.setCurrentPosition(0);
                pitch_steps = deg2step(150, pitch_ratio);
                yaw_homed = DONE;
                //test endstops and try to move away
                if (!digitalRead(yaw_min)) {
                    yaw.move(deg2step(20, yaw_ratio) * yaw_dir);
                    yaw_togo = yaw.distanceToGo();
                    calibrated = STATE2;
                }
                else if (!digitalRead(yaw_max)) {
                    yaw.move(deg2step(-20, yaw_ratio) * yaw_dir);
                    yaw_togo = yaw.distanceToGo();
                    calibrated = STATE2;
                }
                //go to one endstop
                else {
                    yaw.move(deg2step(360, yaw_ratio) * yaw_dir);
                    yaw_togo = yaw.distanceToGo();
                    calibrated = STATE3;
                }
            }
            //wrong direction
            else if (!pitch_togo){
                pitch_dir = -pitch_dir;
                pitch.move(deg2step(-180, pitch_ratio) * pitch_dir);
                pitch_togo = pitch.distanceToGo();
            }
        }
        //move away from endstop and check if they are still triggered
        if (calibrated == STATE2 && !yaw_togo) {
            //still triggered -> wrong direction
            if (!digitalRead(yaw_min)) {
                yaw_dir = -yaw_dir;
                yaw.setCurrentPosition(0);
                yaw.move(deg2step(50, yaw_ratio) * yaw_dir);
                yaw_togo = yaw.distanceToGo();
                calibrated = STATE4;
            }
            else if (!digitalRead(yaw_max)) {
                yaw_dir = -yaw_dir;
                yaw.setCurrentPosition(0);
                yaw.move(deg2step(-50, yaw_ratio) * yaw_dir);
                yaw_togo = yaw.distanceToGo();
                calibrated = STATE4;
            }
            //good direction -> finish the move
            //current position is possitive -> continue to max
            else if (yaw.currentPosition() * yaw_dir > 0) {
                yaw.move(deg2step(30, yaw_ratio) * yaw_dir);
                yaw_togo = yaw.distanceToGo();
                calibrated = STATE4;
            }
            else {
                yaw.move(deg2step(-30, yaw_ratio) * yaw_dir);
                yaw_togo = yaw.distanceToGo();
                calibrated = STATE4;
            }
        }
        //hit at least one endstop
        if (calibrated == STATE3) {
            //wrong direction
            if (!digitalRead(yaw_min)) {
                yaw_dir = -yaw_dir;
                yaw.setCurrentPosition(0);
                yaw.move(deg2step(50, yaw_ratio) * yaw_dir);
                yaw_togo = yaw.distanceToGo();
                calibrated = STATE4;
            }
            //hit max endstop
            else if (!digitalRead(yaw_max)) {
                yaw.setCurrentPosition(0);
                yaw.move(deg2step(-50, yaw_ratio) * yaw_dir);
                yaw_togo = yaw.distanceToGo();
                calibrated = STATE4;
            }
        }
        if (calibrated == STATE4 && !yaw_togo) {
            if (yaw.currentPosition() * yaw_dir > 0)
                yaw.move(deg2step(310, yaw_ratio) * yaw_dir);
            else
                yaw.move(deg2step(-310, yaw_ratio) * yaw_dir);
            
            yaw_togo = yaw.distanceToGo();
            calibrated = STATE5;
        }
        //get pitch steps
        if (calibrated == STATE5) {
            if (!digitalRead(yaw_min) || !digitalRead(yaw_max)) {
                yaw_steps = abs(yaw.currentPosition());
                printf("curr: %d\n", yaw.currentPosition());
                yaw.setCurrentPosition(-yaw.currentPosition() / 2 * yaw_dir);
                printf("YAW POS: %ld\n", yaw.currentPosition());
                yaw.moveTo(0);
                yaw_togo = yaw.distanceToGo();
                printf("togo: %ld\n", yaw_togo);

                calibrated = STATE6;
            }
        }
        if (calibrated == STATE6 && !yaw_togo)
            calibrated = DONE;


        if (!pitch_togo || pitch_homed == NONE)
            goto skip_pitch;

        switch (pitch_homed) {
            case STATE1:
                if (pitch_togo * pitch_dir < 0 && !digitalRead(pitch_min)) {
                    pitch_homed = DONE;
                    pitch.setCurrentPosition(0);
                    pitch.moveTo(0);
                }
                pitch.setSpeed(pitch_speed);
                pitch.runSpeedToPosition();
                break;
            case DONE:
                //end stop hit or at limit
                if ((pitch_togo * pitch_dir < 0 && !digitalRead(pitch_min)) || (pitch_togo * pitch_dir > 0 && pitch.currentPosition() * pitch_dir >= pitch_steps)) {
                    pitch.move(0);
                }
                pitch.setSpeed(pitch_speed);
                pitch.runSpeedToPosition();
                break;
            default: break;
        }

        skip_pitch:
        if (!yaw_togo)
            return;

        switch (yaw_homed) {
            case STATE1:
                if (yaw_togo * yaw_dir > 0 && !digitalRead(yaw_max)) {
                    yaw_homed = DONE;
                    yaw.setCurrentPosition(yaw_steps / 2 * yaw_dir);
                    yaw.moveTo(0);
                }
                yaw.setSpeed(yaw_speed);
                yaw.runSpeedToPosition();
                break;
            case DONE:
                //endstop hit
                if ((yaw_togo * yaw_dir > 0 && !digitalRead(yaw_max)) || (yaw_togo * yaw_dir < 0 && !digitalRead(yaw_min))) {
                    printf("%d %d %d %d\n", yaw_togo, yaw_dir, !digitalRead(yaw_max), !digitalRead(yaw_min));
                    yaw.move(0);
                }
                yaw.setSpeed(yaw_speed);
                yaw.runSpeedToPosition();
                break;
            default: break;
        }
    }
};
