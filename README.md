# antenna-tracker
Simple antenna tracker for RC models.

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
- [ ] Implement tracking
  - [ ] GPS
  - [ ] Compass
  - [ ] Baro
  - [ ] Gyro
- [ ] Basic controlls (home, offset, ...)
- [ ] Basic interface (OLED/LCD)

# Implementation
- Steppers
    - Modular speed
- Endstops
    - Homing
- Getting data from radio
    - UART
    - USB VCP
    - SBUS?
- GPS comms
    - Get data from GPS
- Interface
    - Simple OLED interface
    - 5 way joystick or 3 buttons

# Communication
### Protocol
ACTION DATA ACTION2 DATA2 ...
### Messages
#### Mandatory (always sent)
- UAV_LAT D.M.S
- UAV_LON D.M.S
- UAV_ALT HEIGHT_IN_M
- ARMED BOOL

#### Required if not GPS is connected (sent on ARM)
- HOME_LAT D.M.S
- HOME_LON D.M.S
- HOME_ALT HEIGHT_IN_M

#### Manual controll (same as buttons, sent when needed)
- MANUAL_YAW DEG
- MANUAL_PITCH DEG
- HOME

# Config
Direction for pitch and yaw
