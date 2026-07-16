## Receiver Channel Assignment

The controller reads six PWM channels from the FlySky receiver. The channel mapping used in this project is shown below.

| Receiver Channel | Arduino Pin | Transmitter Control | Used For |
|------------------|-------------|---------------------|-----------|
| CH1 | A0 | VR-B | Auxiliary input |
| CH2 | A1 | VR-A | Auxiliary input |
| CH3 | A2 | Left Stick (Horizontal) | Gripper |
| CH4 | A3 | Left Stick (Vertical) | Throttle |
| CH5 | A4 | Right Stick (Vertical) | Arm |
| CH6 | A5 | Right Stick (Horizontal) | Steering |

---

## Firmware Mapping

Within the firmware, the receiver channels are assigned to robot functions using descriptive pin names.

| Firmware Variable | Arduino Pin | Receiver Channel | Function |
|-------------------|-------------|------------------|----------|
| PIN_STEERING | A5 | CH6 | Steering |
| PIN_ARM | A4 | CH5 | Arm |
| PIN_THROTTLE | A3 | CH4 | Throttle |
| PIN_GRIPPER | A2 | CH3 | Gripper |
| PIN_CH5 | A1 | CH2 | Auxiliary |
| PIN_CH6 | A0 | CH1 | Auxiliary |

This separation keeps the firmware readable while preserving the physical receiver channel numbering.