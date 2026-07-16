## Receiver Channel Assignment

The controller reads six PWM channels from the FlySky receiver. The channel mapping used in this project is shown below.

| Receiver Channel | Arduino Pin | Transmitter Control | Function |
|------------------|-------------|---------------------|----------|
| CH1 | A0 | VR-B | Auxiliary control |
| CH2 | A1 | VR-A | Auxiliary control |
| CH3 | A2 | Left Stick (Horizontal) | Steering |
| CH4 | A3 | Left Stick (Vertical) | Throttle |
| CH5 | A4 | Right Stick (Vertical) | Arm |
| CH6 | A5 | Right Stick (Horizontal) | Gripper |