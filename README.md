# antenna-tracker
Simple antenna tracker for RC models.
Using https://github.com/earlephilhower/arduino-pico (instalation: https://arduino-pico.readthedocs.io/en/latest/platformio.html)


# Positions
Yaw - rotation
- 0° when centered
- Negative = moving clockwise -> MIN
- Positive = moving counter-clockwise -> MAX
Pitch - eleveation
- 0° when at endstop (MIN)
- Positive = moving "up" -> MAX

# TODOs
- [X] Select HW
  - [X] MCU: RP2040
  - [X] Motors: 28byj-48 (will switch to NEMA 11)
  - [X] Sensors: GPS and compass
- [X] Design the tracker
- [ ] Scripts for getting telemetry from EdgeTX
  - [ ] iNav
  - [ ] BetaFlight
  - [ ] Ardupilot
- [ ] Other ways of getting telemetry
- [X] Implement tracking
  - [X] GPS
  - [ ] Compass
- [ ] Basic controlls (home, offset, ...)
- [ ] Basic interface (OLED/LCD)

# Communication
## Protocol
ACTION DATA\n (data are optional for some actions and there can be multiple data for single commnad)
## Messages
### Mandatory (always sent)
- UAV_LAT deg
- UAV_LON deg
- UAV_ALT m
- ARMED bool

### Required if no GPS is connected or other than 3D lock is acquired (sent on ARM)
- HOME_LAT deg
- HOME_LON deg
- HOME_ALT m

### Manual controll
- CALIBRATE
- HOME
- DISABLE
- HOME
- HOME_YAW
- HOME_PITCH
- MOVE_TO pitch_deg yaw_deg
- MOVE_BY pitch_deg yaw_deg

