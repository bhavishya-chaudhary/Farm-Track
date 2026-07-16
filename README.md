# Farm-Track

Farm-Track is an iterative off-road mobile robot project that evolved through three major versions. Rather than redesigning the bot from scratch after each competition, every iteration focused on solving the limitations observed in the previous version.

The project began as a simple wired competition bot and gradually evolved into a wireless robotic platform integrating mechanical design, embedded control, and system-level improvements.

This repository documents the complete engineering journey across all three versions.

> **Repository Focus**
>
> This repository documents the complete evolution of Farm-Track from V1 to V3. However, its primary focus is **FarmTrack V3**, which represents the final and most complete implementation of the project.
>
> V1 and V2 are included to document the engineering journey, design decisions, and mechanical improvements that led to the final version. Since these versions were developed before I began maintaining my projects on GitHub, only their documentation and project history are available. The complete source code, hardware documentation, and implementation details are provided for V3.

---

## Project Evolution

| Version | Highlights |
|---------|------------|
| **[V1](V1/README.md)** | Initial prototype with wired control, four-wheel differential drive, articulated arm, and parallel jaw gripper. |
| **[V2](V2/README.md)** | Higher torque drivetrain, worm gear mechanisms, improved weight distribution, flexible shaft transmission, smoother speed control, and mechanical optimization. |
| **[V3](V3/README.md)** | Wireless control, embedded control logic, cleaner wiring, improved system integration, and drivetrain experimentation. |

---

## Versions

### V1

The first working prototype focused on building a reliable off-road robot capable of object handling using an articulated arm and gripper. During development, several practical issues such as tyre traction and control smoothness were identified and addressed.

**Documentation:** [V1 README](V1/README.md)

*Source code for this version was not preserved.*

---

### V2

The second iteration focused on mechanical improvements. The drivetrain was optimized for higher torque, the arm mechanism was redesigned using a worm-and-wheel drive, the gripper was driven through a flexible shaft, and the overall control became much smoother.

**Documentation:** [V2 README](V2/README.md)

*Source code for this version was not preserved.*

---

### V3

The final iteration integrated wireless control, embedded control logic, improved drivetrain performance, cleaner electronics, and a more organized wiring layout.

**Documentation:** [V3 README](V3/README.md)

The complete implementation, including source code, hardware documentation, and project images, is available for this version.

---

## Technologies Used

- Arduino Uno
- FlySky CT6B & FS-iA6B
- BTS7960 Motor Drivers
- L298N Motor Driver
- Differential Drive
- DC Geared Motors
- Worm Gear Mechanism
- Flexible Shaft Power Transmission
- Embedded C++
- Git & GitHub

---

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

## Images

Unless otherwise stated, all photographs, renders, and project images in this repository are © Bhavishya Chaudhary.

The images are provided to document the development of the project and may not be reproduced, redistributed, or used in other projects without permission.

The source code in this repository remains licensed under the MIT License.

---

## Author

**Bhavishya Chaudhary**

If you have any questions or suggestions, feel free to connect with me on LinkedIn or GitHub.