#include <Arduino.h>
#include <AccelStepper.h>


//TODO add smart homing


class StepperManager {
private:
    /* data */
    AccelStepper pitch;
    AccelStepper yaw;

    uint8_t pitch_homed = 0;
    uint8_t yaw_homed = 0;

    uint8_t pitch_calibrated = 0;
    uint8_t yaw_calibrated = 0;

    int32_t pitch_steps = INT32_MAX;
    int32_t yaw_steps = INT32_MAX;

    int pitch_min = -1;
    int yaw_min = -1;
    int yaw_max = -1;


    int8_t pitch_dir = 1;
    int8_t yaw_dir = 1;

    int pitch_speed = 500;
    int yaw_speed = 500;

    float pitch_ratio = 0;
    float yaw_ratio = 0;


    int16_t deg2step(int deg, float ratio) {
        return deg * 32 * 63.68395 / 360 * ratio;
    }

public:
    StepperManager(uint8_t yaw1, uint8_t yaw2, uint8_t yaw3, uint8_t yaw4, uint8_t pitch1, uint8_t pitch2, uint8_t pitch3, uint8_t pitch4, int yaw_min, int yaw_mx, int pitch_min) {
        yaw = AccelStepper(yaw1, yaw3, yaw2, yaw4);
        pitch = AccelStepper(pitch1, pitch3, pitch2, pitch4);
        this->pitch_min = pitch_min;
        this->yaw_max = yaw_max;
        this->yaw_min = yaw_min;
    }
    ~StepperManager();

    void begin(int pitch_max_speed, int yaw_max_speed, int pitch_speed, int yaw_speed, float pitch_ratio, float yaw_ratio) {
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

    //TODO calibrate

    void calibrate() {
        printf("Calibrating PITCH\n");
        pitch_homed = 2;
        pitch.move(4000 * pitch_dir);
        int togo = pitch.distanceToGo();
        //move yaw in loop until endstop is hit
        while (pitch.distanceToGo()) {
            if (!digitalRead(pitch_min) && abs(togo - pitch.distanceToGo()) > 300)
                break;
            pitch.setSpeed(700 <= pitch.maxSpeed() ? 700 : pitch.maxSpeed());
            pitch.runSpeedToPosition();
        }

        //wrong direction
        if (!digitalRead(pitch_min)) {
            pitch_dir = -1;
            movePitchBy(4000);
        }
        pitch.setCurrentPosition(0);
        movePitchBy(-30000);
        pitch_steps = abs(pitch.currentPosition());
        pitch.setCurrentPosition(0);
        pitch_calibrated = true;

        //yaw
        printf("Calibrating YAW\n");
        yaw_homed = 2;
        yaw.move(60000 * yaw_dir);
        togo = yaw.distanceToGo();
        //move yaw in loop until endstop is hit
        while (yaw.distanceToGo()) {
            if ((!digitalRead(yaw_min) || !digitalRead(yaw_max)) && abs(togo - yaw.distanceToGo()) > 300)
                break;
            yaw.setSpeed(yaw_speed);
            yaw.runSpeedToPosition();
        }

        yaw.setCurrentPosition(0);

        //flip direction if we moved clockwise instead of counter-clockwise and hit MIN endstop
        yaw_dir = !digitalRead(yaw_max) ? 1 : -1;

        if (!digitalRead(yaw_min))
            moveYawBy(60000);
        else if (!digitalRead(yaw_max))
            moveYawBy(-60000);
        yaw_steps = abs(yaw.currentPosition());
        yaw.setCurrentPosition(yaw_steps / 2 * yaw_dir);
        yaw_calibrated = true;
    }

    void home(bool pitch = true, bool yaw = true) {
        if (pitch && !pitch_calibrated)
            printf("PITCH NOT CALIBRATED\n");
        else if (pitch && pitch_calibrated)
            this->pitch.move(-9999999 * pitch_dir);

        if (yaw && !yaw_calibrated)
            printf("YAW NOT CALIBRATED\n");
        else if (yaw && yaw_calibrated)
            this->yaw.move(9999999 * yaw_dir);
    }


    void moveTo(int16_t pitch_angle, int16_t yaw_angle) {
        pitch.moveTo(deg2step(pitch_angle, pitch_ratio) * pitch_dir);
        yaw.moveTo(deg2step(yaw_angle, yaw_ratio) * yaw_dir);
    }

    void moveBy(int16_t pitch_deg, int16_t yaw_deg) {
        pitch.move(deg2step(pitch_deg, pitch_ratio) * pitch_dir);
        yaw.move(deg2step(yaw_deg, yaw_ratio) * yaw_dir);
    }

    /** @brief Main loop for moving the steppers*/
    void run() {
        if (pitch_homed) {
            int togo = pitch.distanceToGo();
            //endstop hit
            if (togo * pitch_dir < 0 && !digitalRead(pitch_min)) {
                //home in progress -> finish it
                if (pitch_homed == 1) {
                    pitch_homed = 2;
                    pitch.setCurrentPosition(0);
                    pitch.moveTo(0);
                }
                else
                    pitch.move(-togo);
                goto skip;
            }
            //virtual endstop hit
            if (togo * pitch_dir > 0 && pitch.currentPosition() * pitch_dir >= pitch_steps)
                pitch.move(-togo);
                goto skip;
            pitch.setSpeed(pitch_speed);
            pitch.runSpeedToPosition();
        }
        skip:
        if (yaw_homed) {
            int togo = yaw.distanceToGo();
            //endstop hit
            if ((togo * yaw_dir > 0 && !digitalRead(yaw_max)) || (togo * yaw_dir < 0 && !digitalRead(yaw_min))) {
                //home in progress -> finish it
                if (yaw_homed == 1) {
                    yaw_homed = 2;
                    yaw.setCurrentPosition(yaw_steps / 2 * yaw_dir);
                    yaw.moveTo(0);
                }
                else
                    yaw.move(-togo);
                return;
            }
            yaw.setSpeed(pitch_speed);
            yaw.runSpeedToPosition();
        }

        return yaw.
    }
};
