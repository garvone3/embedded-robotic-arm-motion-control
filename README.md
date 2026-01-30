# Coordinated Multi-Joint Robotic Arm Controller

## Overview

This project implements a hardware-based motion control system for a 3-4 DOF serial manipulator using an Arduino microcontroller and hobby servo actuators. The system demonstrates fundamental embedded robotics concepts including coordinated motion planning, real-time interpolation, safety constraint enforcement, and deterministic control loop execution.

**Target Hardware:**
- Microcontroller: Arduino UNO/Nano (ATmega328P, 16MHz)
- Actuators: 3-4 RC servos (e.g., SG90, MG995)
- Power: External 5V regulated supply for servos (1-2A minimum)
- Interface: Serial UART (115200 baud)

---

## System Architecture

### Control Architecture

```
┌─────────────────────────────────────────────────────┐
│  Application Layer (robotic_arm_control.ino)       │
│  - Pose sequencing (FSM)                            │
│  - Serial command parser                            │
│  - Safety monitoring                                │
└────────────────┬────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────┐
│  Motion Control Layer (MotionController class)      │
│  - Coordinated joint interpolation                  │
│  - Velocity limiting                                │
│  - Soft/hard limit enforcement                      │
│  - Emergency stop handling                          │
└────────────────┬────────────────────────────────────┘
                 │
┌────────────────▼────────────────────────────────────┐
│  Hardware Abstraction (Arduino Servo library)       │
│  - PWM generation (50Hz, 1-2ms pulse width)         │
│  - Timer-based servo control                        │
└─────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Separation of Concerns**: Motion planning (application) is decoupled from motion execution (controller) and hardware actuation.

2. **Non-Blocking Execution**: The motion controller uses incremental position updates rather than blocking delays, allowing the main loop to remain responsive.

3. **Coordinated Motion**: All joints move simultaneously with synchronized interpolation, rather than sequential joint-by-joint motion.

4. **Safety-First Design**: Multiple layers of protection prevent mechanical damage and unsafe operation.

---

## Core Features

### 1. Coordinated Multi-Joint Interpolation

**Problem:** RC servos have no internal position feedback. Commanding multiple servos to new positions simultaneously results in uncoordinated, jerky motion with race conditions.

**Solution:** Software-based linear interpolation:
- Target pose is set, but not executed immediately
- Each control cycle, all joints increment toward target by a small step
- All joints arrive at target simultaneously (synchronized)
- Motion speed is adjustable (degrees per update cycle)

**Implementation:**
```cpp
// Each joint updates incrementally in coordinated fashion
void MotionController::updateJoint(int jointIndex) {
  int delta = targetAngle - currentAngle;
  int step = (abs(delta) <= interpolationSpeed) ? 
             delta : 
             (delta > 0 ? interpolationSpeed : -interpolationSpeed);
  currentAngle += step;
  servo->write(currentAngle);
}
```

### 2. Multi-Layer Safety System

**Hard Limits** (Mechanical Protection):
- Absolute min/max angles enforced at constraint level
- Prevents commands outside physical range of motion
- Protects against mechanical collision/damage

**Soft Limits** (Operational Envelope):
- Conservative range (5° margin from hard limits)
- Normal operation stays within soft limits
- Provides buffer for calibration errors

**Velocity Limiting**:
- Per-joint maximum velocity constraints
- Prevents excessive angular acceleration
- Reduces mechanical stress and current spikes

**Emergency Stop**:
- Immediate motion halt with state preservation
- No abrupt servo commands (reduces wear)
- Requires explicit reset before resuming operation

### 3. Timing Determinism

**Fixed Update Rate:**
- Main loop executes at 50Hz (20ms period)
- Predictable motion behavior
- Enables velocity/acceleration analysis

**Cycle Time Monitoring:**
- Measures actual control loop duration
- Detects timing violations or computational overload
- Critical for real-time system validation

**Why This Matters:**
In production robotics, timing jitter causes:
- Non-smooth motion profiles
- Velocity/acceleration deviations
- Potential resonance excitation
- Difficult-to-debug behavior

### 4. Calibration Framework

**Per-Joint Calibration Offsets:**
- Compensates for servo horn mounting errors
- Corrects for mechanical alignment issues
- Separates logical angles from hardware angles

**Calibration Process** (recommended):
1. Mount arm in known reference position
2. Measure angular error for each joint
3. Record offset in `CALIBRATION_OFFSETS[]` array
4. Recompile and upload

---

## Engineering Trade-offs

### Design Decisions

| Decision | Rationale | Trade-off |
|----------|-----------|-----------|
| **No position feedback** | RC servos lack encoders | Open-loop: no error correction |
| **Linear interpolation** | Computationally cheap for 8-bit MCU | Not time-optimal or smooth acceleration |
| **Fixed update rate (delay)** | Simple, deterministic timing | Not truly real-time; sensitive to code changes |
| **Soft limits in software** | No additional hardware cost | Relies on correct configuration |
| **Discrete angles (int)** | Memory-efficient on Arduino | Limited resolution (~0.5° with standard servos) |

### Limitations and Known Issues

1. **No Feedback Loop:**
   - System assumes servos reach commanded position
   - No detection of mechanical binding or slippage
   - Cannot compensate for external forces or load variation
   - **Mitigation:** Use servos with adequate torque margin

2. **Timing Non-Determinism:**
   - `delay(20)` is not a real-time guarantee
   - Serial communication and other tasks cause jitter
   - No deadline enforcement mechanism
   - **Impact:** Slight velocity variations; acceptable for demonstration
   - **Production Alternative:** Timer interrupts or RTOS

3. **No Inverse Kinematics:**
   - Joint angles must be specified directly
   - No Cartesian endpoint control
   - Task-space planning must be done externally
   - **Scope:** This is a motion controller, not a motion planner

4. **Memory Constraints:**
   - ATmega328P: 2KB RAM, 32KB Flash
   - Limited pose storage capacity
   - No dynamic trajectory buffering
   - **Implication:** Complex sequences must be hardcoded or streamed

5. **Servo Limitations:**
   - Hobby servos: ~180° range, 0.5-1° deadband
   - No velocity/current feedback
   - Stall protection depends on internal electronics
   - Non-backdrivable (high friction when unpowered)

6. **Power Supply Considerations:**
   - Servos draw significant current during motion (1-2A)
   - Arduino cannot power servos from 5V pin
   - Inadequate supply causes brown-outs and erratic behavior
   - **Requirement:** Dedicated regulated 5V supply, shared ground

---

## What This Project Demonstrates

### Embedded Systems Competencies

- **Real-time control loops** with deterministic timing
- **State machine design** for task sequencing
- **Modular software architecture** (separation of concerns)
- **Hardware abstraction** for portability
- **Resource-constrained programming** (2KB RAM budget)

### Robotics Fundamentals

- **Coordinated multi-axis motion control**
- **Software-based trajectory generation** (interpolation)
- **Safety constraint enforcement** (limits, e-stop)
- **Calibration methodology** for mechanical systems
- **Serial communication protocols** for telemetry/commands

### Professional Practices

- **Defensive programming** (bound checking, validation)
- **Diagnostic instrumentation** (timing, status reporting)
- **Code documentation** (clear comments, function contracts)
- **Version control readiness** (modular files, no binary artifacts)
- **Honest scope definition** (no overclaiming capabilities)

---

## Hardware Setup

### Wiring Diagram

```
Arduino UNO/Nano          Servos              Power Supply
┌──────────────┐         ┌──────┐           ┌──────────────┐
│              │         │      │           │  5V 2A       │
│          D9  ├─────────┤ Base │           │  Regulated   │
│          D10 ├─────────┤Shldr │           └──┬───────┬───┘
│          D11 ├─────────┤Elbow │              │       │
│          D6  ├─────────┤Grip  │              │       │
│              │         └──┬───┘              │       │
│          GND ├────────────┴──────────────────┘       │
│              │                                       │
│          VIN ├───────────────────────────────────────┘
└──────────────┘           (Optional: power Arduino from same supply)
```

**Critical:** Servos and Arduino must share a common ground. Failure to do so results in erratic servo behavior or damage.

### Bill of Materials

| Component | Specification | Quantity | Notes |
|-----------|---------------|----------|-------|
| Arduino UNO/Nano | ATmega328P | 1 | UNO preferred for easier debugging |
| RC Servo (small) | SG90 or equivalent | 3-4 | Gripper and upper joints |
| RC Servo (medium) | MG995 or equivalent | 0-1 | Base (if high torque needed) |
| Power Supply | 5V, 2A regulated | 1 | Must be stable under load |
| Jumper Wires | 22 AWG | ~10 | Signal and power distribution |
| Breadboard | Half-size | 1 | Optional: for prototyping connections |

---

## Software Setup

### Prerequisites

- Arduino IDE 1.8.x or 2.x
- Arduino AVR Boards package (included by default)
- Servo library (built-in)

### Installation

1. **Clone or download project files:**
   ```
   robotic_arm_control.ino
   motion_controller.h
   motion_controller.cpp
   ```

2. **Open `robotic_arm_control.ino` in Arduino IDE**

3. **Configure joint limits** (lines 27-35):
   - Adjust `*_MIN` and `*_MAX` based on your mechanical design
   - Conservative limits prevent collisions during testing

4. **Set calibration offsets** (line 41):
   - Initially leave as `{0, 0, 0, 0}`
   - Calibrate after first test run (see Calibration section)

5. **Upload to Arduino:**
   - Board: Arduino UNO (or Nano)
   - Port: Select appropriate COM/USB port
   - Upload

---

## Usage

### Serial Commands

Connect to Arduino via Serial Monitor (115200 baud):

| Command | Function |
|---------|----------|
| `START` | Begin automatic motion sequence |
| `STOP` | Halt sequence (emergency stop) |
| `HOME` | Return to home position |
| `ESTOP` | Emergency stop all motion |
| `RESET` | Clear emergency stop flag |
| `STATUS` | Print joint angles and safety status |
| `DIAG` | Print timing diagnostics |
| `HELP` | Display command list |

### Typical Workflow

1. **Power on system** (external supply first, then Arduino)
2. **Open Serial Monitor** (115200 baud)
3. **Wait for "Initialization complete"**
4. **Send `STATUS`** to verify joint positions
5. **Send `START`** to run demo sequence
6. **Monitor motion** (verify smooth, coordinated movement)
7. **Send `ESTOP`** if any issues occur
8. **Send `RESET`** to recover from e-stop

### Calibration Procedure

1. **Mount arm in known reference configuration**
   - Typically all joints at 90° (mid-range)
   - Use protractor or angle gauge

2. **Run system and send `STATUS` command**

3. **Measure angular error for each joint**
   - Error = Actual angle - Commanded angle

4. **Update `CALIBRATION_OFFSETS[]` in `.ino` file:**
   ```cpp
   const int CALIBRATION_OFFSETS[NUM_JOINTS] = {-5, 3, 0, -2};
   ```

5. **Recompile and upload**

6. **Verify:** Send `STATUS`, check angles match physical configuration

---

## Extending This Project

### Realistic Enhancements

**1. External Trajectory Streaming:**
- Receive pose sequences via serial
- Circular buffer for smooth continuous motion
- Enables integration with PC-based planner
- **Complexity:** Moderate (requires protocol design)

**2. Current Monitoring:**
- Add ACS712 current sensor
- Detect motor stall conditions
- Implement load-based safety cutoffs
- **Hardware:** ~$2 per channel

**3. Potentiometer-Based Joint Feedback:**
- Add rotary potentiometers to joints
- Closed-loop position control
- Enables compliance detection
- **Hardware:** $1-3 per joint, requires ADC pins

**4. Trajectory Optimization:**
- Replace linear interpolation with trapezoidal velocity profile
- Smoother acceleration/deceleration
- Reduced mechanical stress
- **Complexity:** Moderate (requires velocity planning)

**5. Workspace Limits:**
- Define Cartesian workspace boundaries
- Forward kinematics to check endpoint position
- Reject poses that violate workspace
- **Complexity:** High (requires kinematics implementation)

### Not Recommended (Scope Creep)

- ❌ **ROS integration:** Overhead too high for Arduino
- ❌ **Machine learning:** No floating-point capability
- ❌ **Vision-based control:** No camera interface
- ❌ **Inverse kinematics:** Computationally expensive for 8-bit MCU
- ❌ **Force control:** No force sensors in design

---

## Troubleshooting

### Servos Not Moving

**Symptom:** Serial messages appear, but no motion

**Causes:**
1. Insufficient power supply (brown-out)
2. Loose signal wire connection
3. Ground not shared between Arduino and servos
4. Emergency stop active

**Diagnosis:**
- Check power supply voltage under load (should be 4.8-5.2V)
- Send `STATUS` command - verify emergency stop is inactive
- Swap servo to known-good unit

### Jerky or Erratic Motion

**Symptom:** Servos move, but motion is not smooth

**Causes:**
1. Main loop timing too slow (check diagnostics)
2. Interpolation speed too high
3. Serial communication causing jitter
4. Mechanical binding

**Diagnosis:**
- Send `DIAG` - verify loop rate is ~50Hz
- Reduce `interpolationSpeed` (line 80, 97, etc.)
- Minimize serial prints during motion

### System Crashes or Resets

**Symptom:** Arduino restarts unexpectedly

**Causes:**
1. Power supply brown-out (most common)
2. Watchdog timer triggering (if enabled)
3. Stack overflow (unlikely with this codebase)

**Diagnosis:**
- Monitor Arduino 5V pin with multimeter during motion
- Add 1000µF capacitor across servo power supply
- Reduce number of simultaneous moving joints

---

## File Structure

```
robotic_arm_control/
├── robotic_arm_control.ino    # Main application (FSM, serial interface)
├── motion_controller.h         # Motion controller class declaration
└── motion_controller.cpp       # Motion controller implementation
```

**Design Note:** Header/implementation split enables unit testing (with mocking) and facilitates migration to more powerful platforms (e.g., Teensy, STM32).

---

## Testing and Validation

### Functional Tests

1. **Joint Limit Enforcement:**
   - Command angles outside limits
   - Verify clamping to safe range
   - Check serial warnings

2. **Emergency Stop:**
   - Initiate motion sequence
   - Send `ESTOP` command mid-motion
   - Verify immediate halt
   - Confirm system requires `RESET`

3. **Coordinated Motion:**
   - Command multi-joint move
   - Observe all joints moving simultaneously
   - Verify synchronized arrival at target

4. **Calibration:**
   - Set known offsets
   - Verify hardware angles match commanded angles
   - Test with various poses

### Performance Metrics

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Loop rate | 48-52 Hz | `DIAG` command output |
| Position accuracy | ±2° | Protractor measurement |
| Motion smoothness | No jerks | Visual observation |
| E-stop latency | <100ms | Stopwatch verification |

---

## Academic Context

This project is suitable for:

- **Embedded Systems Courses:** Real-time control, resource constraints
- **Robotics Courses:** Motion planning basics, safety systems
- **Mechatronics Courses:** Hardware/software integration
- **Capstone Projects:** Foundation for more complex manipulator work

**Learning Outcomes:**
1. Implement coordinated multi-axis motion control
2. Apply safety constraints in embedded systems
3. Design modular software architectures
4. Analyze timing determinism in control loops
5. Debug hardware/software integration issues

---

## References and Further Reading

### Fundamental Concepts

1. **Motion Interpolation:**
   - Craig, J.J. (2005). *Introduction to Robotics: Mechanics and Control*. Chapter 7.
   
2. **Real-Time Systems:**
   - Kopetz, H. (2011). *Real-Time Systems: Design Principles for Distributed Embedded Applications*.

3. **Servo Control:**
   - Arduino Servo Library Documentation: https://www.arduino.cc/en/Reference/Servo
   - PWM fundamentals for RC servos

### Advanced Topics (Beyond This Project)

- **Trajectory Optimization:** Minimum-jerk, minimum-time planning
- **Kinematics:** Forward/inverse kinematics for serial manipulators (Denavit-Hartenberg convention)
- **Dynamics:** Joint torque calculation, gravity compensation
- **Compliance:** Series elastic actuation, impedance control

---

## License

This project is provided as educational reference material. Use at your own risk. No warranty of fitness for any purpose is provided.

**Safety Disclaimer:** This system controls electromechanical actuators. Improper use can cause equipment damage or injury. Always:
- Test in a clear workspace
- Use appropriate power supplies
- Implement emergency stop procedures
- Supervise operation at all times

---

## Contributing

This project is designed as a self-contained educational example. If you extend this work for academic purposes, consider:

- Documenting your modifications clearly
- Maintaining the modular architecture
- Adding unit tests for new functionality
- Sharing insights on hardware limitations encountered

---

## Acknowledgments

Developed as a demonstration of embedded robotics fundamentals for graduate-level robotics education (Erasmus Mundus IFROS program context). Emphasis on honest capability representation and professional engineering practices.

**Version:** 2.0  
**Last Updated:** January 2025  
**Contact:** Submit issues or questions through your academic advisor or program coordinator.
