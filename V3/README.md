# FarmTrack V3

<p align="center">
  <img src="Images/Top_View.jpeg" alt="FarmTrack V3" width="250">
</p>

FarmTrack V3 is the third and final iteration of the Farm-Track project.

This version focused on bringing together everything learned from the previous two iterations into a single platform. The primary objectives were higher speed, wireless control, better system integration, cleaner design, and an improved driving experience.

This directory contains the Arduino source code, hardware connections, and documentation for the V3 control system.

> **Note**
>
> The complete evolution of the project, including V1 and V2, can be found in the main **Farm-Track** repository.

---

## Features

- Four-wheel differential drive
- Wireless control using FlySky CT6B transmitter and FS-iA6B receiver
- Arduino-based control system
- Differential drive channel mixing
- Independent control of drivetrain, arm, and gripper
- Receiver calibration stored in EEPROM
- Automatic failsafe on signal loss
- Deadband compensation for smoother control
- Quadratic throttle response with non-linear steering
- On-board calibration using a push button
- Serial debugging for receiver monitoring

---

## Hardware

| Component | Description |
|-----------|-------------|
| Controller | Arduino Uno |
| Radio System | FlySky CT6B + FS-iA6B Receiver |
| Drive Motor Drivers | 2 × BTS7960 |
| Arm & Gripper Driver | L298N Dual H-Bridge |
| Drive Motors | 8 × 12V DC Geared Motors (paired configuration) |
| Arm Motor | 12V DC Geared Motor |
| Gripper Motor | 12V DC Geared Motor |
| Battery | 3S LiPo |
| User Interface | Push Button and Status LED |

---

## Project Gallery

### Complete Bot

<p align="center">
  <img src="Images/Top_View.jpeg" alt="FarmTrack V3" width="200">
</p>

---

### Electronics Layout

<p align="center">
  <img src="Images/Closeup_Circuit.jpeg" alt="Electronics Layout" width="150">
</p>

---

## Software Overview

The controller continuously reads six PWM channels from the FlySky receiver using pin change interrupts.

Each receiver channel is calibrated independently and stored in EEPROM, allowing the controller to compensate for transmitter variations without modifying the source code.

Joystick inputs are processed through dead zones and response curves before generating differential drive commands for the left and right sides of the bot.

The drivetrain, arm, and gripper are controlled independently, allowing smooth operation of all mechanisms.

If receiver communication is lost for more than 500 ms, the failsafe immediately stops every motor.

---

## Receiver Channel Assignment

The controller reads six PWM channels from the FlySky receiver using the following mapping.

| Receiver Channel | Arduino Pin | Transmitter Control | Used For |
|------------------|-------------|---------------------|-----------|
| CH1 | A0 | VR-B | Auxiliary input |
| CH2 | A1 | VR-A | Auxiliary input |
| CH3 | A2 | Left Stick (Horizontal) | Gripper |
| CH4 | A3 | Left Stick (Vertical) | Throttle |
| CH5 | A4 | Right Stick (Vertical) | Arm |
| CH6 | A5 | Right Stick (Horizontal) | Steering |

The firmware assigns each receiver channel to a specific robot function, allowing the transmitter layout to remain independent of the control logic.

---

---

## Control Architecture

The following diagram shows the high-level control architecture of FarmTrack V3.

```mermaid
flowchart TD

    TX["FlySky CT6B<br/>Transmitter"]
    RX["FS-iA6B<br/>Receiver"]

    ARD["Arduino Uno"]

    BTS1["BTS7960<br/>Left Drive"]
    BTS2["BTS7960<br/>Right Drive"]

    L298["L298N<br/>Arm & Gripper"]

    LEFT["Left Drive Motors"]
    RIGHT["Right Drive Motors"]

    ARM["Arm Motor"]
    GRIP["Gripper Motor"]

    TX --> RX
    RX --> ARD

    ARD --> BTS1
    ARD --> BTS2
    ARD --> L298

    BTS1 --> LEFT
    BTS2 --> RIGHT

    L298 --> ARM
    L298 --> GRIP
```

## Future Improvements

Some improvements I would like to explore in future versions include:

- Closed-loop speed control using wheel encoders
- Current sensing for drivetrain protection
- IMU-assisted driving
- Custom PCB for motor control and wiring
- Modular wiring harness
- Brushless drivetrain

---

## License

This project is released under the MIT License.

---

## Author

**Bhavishya Chaudhary**

If you have any questions or suggestions, feel free to connect with me on LinkedIn or GitHub.