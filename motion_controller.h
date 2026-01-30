/*
 * Motion Controller Header
 * Coordinated multi-joint motion with interpolation
 * 
 * Features:
 * - Hardware-enforced joint limits with soft limit margin
 * - Velocity limiting for safe operation
 * - Emergency stop capability
 * - Timing determinism tracking
 * - Per-joint calibration offsets
 */

#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

#define NUM_JOINTS 4
#define MAX_JOINTS 4
#define SOFT_LIMIT_MARGIN 5  // degrees from hard limit

// Joint configuration structure
struct JointConfig {
  Servo* servo;
  int minAngle;           // Hard minimum (mechanical limit)
  int maxAngle;           // Hard maximum (mechanical limit)
  int softMinAngle;       // Software minimum (safe operating range)
  int softMaxAngle;       // Software maximum (safe operating range)
  int currentAngle;       // Current position (degrees)
  int targetAngle;        // Target position (degrees)
  int calibrationOffset;  // Zero-position calibration offset
  int maxVelocity;        // Maximum allowed velocity (degrees/sec)
  bool isActive;
};

class MotionController {
  public:
    MotionController();
    
    // Servo management
    bool addServo(Servo* servo, int minAngle, int maxAngle);
    bool setCalibrationOffset(int jointIndex, int offset);
    
    // Motion control
    void setTargetAngles(const int* angles);
    void setTargetAngle(int jointIndex, int angle);
    void setInterpolationSpeed(int degreesPerStep);
    void setMaxVelocity(int jointIndex, int maxDegreesPerSec);
    
    // Safety
    void emergencyStop();
    void resetEmergencyStop();
    bool isEmergencyStopped();
    bool validatePose(const int* angles);
    
    // Execution
    void update();
    void executeImmediate();
    
    // Status and diagnostics
    bool isMoving();
    int getCurrentAngle(int jointIndex);
    unsigned long getLastUpdateTime();
    unsigned long getUpdateCycleTime();
    bool isWithinSoftLimits(int jointIndex);
    
  private:
    JointConfig joints[MAX_JOINTS];
    int activeJointCount;
    int interpolationSpeed;      // degrees per update step
    bool moving;
    bool emergencyStopActive;
    unsigned long lastUpdateTime;
    unsigned long cycleStartTime;
    unsigned long lastCycleTime;
    
    // Helper functions
    int constrainAngle(int jointIndex, int angle);
    int applySoftLimits(int jointIndex, int angle);
    void updateJoint(int jointIndex);
    bool allJointsAtTarget();
    void enforceVelocityLimits(int jointIndex, int* step);
};

#endif
