# Farm-Track

Farm-Track is an iterative off-road mobile robot project that evolved through three major versions. Rather than redesigning the bot from scratch after each competition, every iteration focused on solving the limitations observed in the previous version.

The project began as a simple wired competition bot and gradually evolved into a wireless robotic platform integrating mechanical design, embedded control, and system-level improvements.

This repository documents the complete engineering journey across all three versions.

---

## Project Evolution

| Version | Highlights |
|---------|------------|
| **V1** | Initial prototype with wired control, four-wheel differential drive, articulated arm, and parallel jaw gripper. |
| **V2** | Higher torque drivetrain, worm gear mechanisms, improved weight distribution, flexible shaft transmission, smoother speed control, and mechanical optimization. |
| **V3** | Wireless control, embedded control logic, cleaner wiring, improved system integration, and drivetrain experimentation. |

---

## Repository Structure

```
Farm-Track
│
├── README.md
├── LICENSE
│
├── V1
│   └── README.md
│
├── V2
│   └── README.md
│
└── V3
    ├── README.md
    ├── CONNECTIONS.md
    ├── FarmTrack_V3.ino
    └── Images
```

---

## Versions

### V1

The first working prototype focused on building a reliable off-road robot capable of object handling using an articulated arm and gripper. During development, several practical issues such as tyre traction and control smoothness were identified and addressed.

➡️ See **V1/README.md**

---

### V2

The second iteration focused on mechanical improvements. The drivetrain was optimized for higher torque, the arm mechanism was redesigned using a worm-and-wheel drive, the gripper was driven through a flexible shaft, and the overall control became much smoother.

➡️ See **V2/README.md**

---

### V3

The final iteration integrated wireless control, embedded control logic, improved drivetrain performance, cleaner electronics, and a more organized wiring layout.

➡️ See **V3/README.md**

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

## Author

**Bhavishya Chaudhary**

If you have any questions or suggestions, feel free to connect with me on LinkedIn or GitHub.