/*
 * Motion Controller Implementation
 * Smooth interpolated motion for coordinated multi-joint control
 * 
 * Safety features:
 * - Hard and soft joint limits
 * - Emergency stop with state preservation
 * - Velocity limiting per joint
 * - Timing diagnostics for control loop analysis
 */

#include "motion_controller.h"

MotionController::MotionController() {
  activeJointCount = 0;
  interpolationSpeed = 2; // default: 2 degrees per step
  moving = false;
  emergencyStopActive = false;
  lastUpdateTime = 0;
  cycleStartTime = 0;
  lastCycleTime = 0;
  
  // Initialize joint array
  for (int i = 0; i < MAX_JOINTS; i++) {
    joints[i].servo = nullptr;
    joints[i].minAngle = 0;
    joints[i].maxAngle = 180;
    joints[i].softMinAngle = SOFT_LIMIT_MARGIN;
    joints[i].softMaxAngle = 180 - SOFT_LIMIT_MARGIN;
    joints[i].currentAngle = 90;
    joints[i].targetAngle = 90;
    joints[i].calibrationOffset = 0;
    joints[i].maxVelocity = 60; // default: 60 deg/sec
    joints[i].isActive = false;
  }
}

bool MotionController::addServo(Servo* servo, int minAngle, int maxAngle) {
  if (activeJointCount >= MAX_JOINTS || servo == nullptr) {
    return false;
  }
  
  int index = activeJointCount;
  joints[index].servo = servo;
  joints[index].minAngle = minAngle;
  joints[index].maxAngle = maxAngle;
  joints[index].softMinAngle = minAngle + SOFT_LIMIT_MARGIN;
  joints[index].softMaxAngle = maxAngle - SOFT_LIMIT_MARGIN;
  joints[index].currentAngle = 90; // default middle position
  joints[index].targetAngle = 90;
  joints[index].isActive = true;
  
  activeJointCount++;
  return true;
}

bool MotionController::setCalibrationOffset(int jointIndex, int offset) {
  if (jointIndex >= 0 && jointIndex < activeJointCount && joints[jointIndex].isActive) {
    joints[jointIndex].calibrationOffset = offset;
    return true;
  }
  return false;
}

void MotionController::setTargetAngles(const int* angles) {
  if (emergencyStopActive) {
    return; // Reject motion commands during emergency stop
  }
  
  for (int i = 0; i < activeJointCount; i++) {
    if (joints[i].isActive) {
      int constrainedAngle = constrainAngle(i, angles[i]);
      joints[i].targetAngle = applySoftLimits(i, constrainedAngle);
    }
  }
  moving = true;
}

void MotionController::setTargetAngle(int jointIndex, int angle) {
  if (emergencyStopActive) {
    return;
  }
  
  if (jointIndex >= 0 && jointIndex < activeJointCount && joints[jointIndex].isActive) {
    int constrainedAngle = constrainAngle(jointIndex, angle);
    joints[jointIndex].targetAngle = applySoftLimits(jointIndex, constrainedAngle);
    moving = true;
  }
}

void MotionController::setInterpolationSpeed(int degreesPerStep) {
  if (degreesPerStep > 0) {
    interpolationSpeed = degreesPerStep;
  }
}

void MotionController::setMaxVelocity(int jointIndex, int maxDegreesPerSec) {
  if (jointIndex >= 0 && jointIndex < activeJointCount && 
      joints[jointIndex].isActive && maxDegreesPerSec > 0) {
    joints[jointIndex].maxVelocity = maxDegreesPerSec;
  }
}

void MotionController::emergencyStop() {
  emergencyStopActive = true;
  moving = false;
  // Current positions are preserved - no sudden movements
}

void MotionController::resetEmergencyStop() {
  emergencyStopActive = false;
}

bool MotionController::isEmergencyStopped() {
  return emergencyStopActive;
}

bool MotionController::validatePose(const int* angles) {
  // Check if all angles are within safe limits
  for (int i = 0; i < activeJointCount; i++) {
    if (joints[i].isActive) {
      int constrainedAngle = constrainAngle(i, angles[i]);
      if (constrainedAngle < joints[i].softMinAngle || 
          constrainedAngle > joints[i].softMaxAngle) {
        return false;
      }
    }
  }
  return true;
}

void MotionController::update() {
  cycleStartTime = micros();
  
  if (!moving || emergencyStopActive) {
    return;
  }
  
  // Update all active joints simultaneously (coordinated motion)
  for (int i = 0; i < activeJointCount; i++) {
    if (joints[i].isActive) {
      updateJoint(i);
    }
  }
  
  // Check if all joints reached target
  if (allJointsAtTarget()) {
    moving = false;
  }
  
  lastUpdateTime = micros();
  lastCycleTime = lastUpdateTime - cycleStartTime;
}

void MotionController::executeImmediate() {
  if (emergencyStopActive) {
    return;
  }
  
  // Move all joints to target immediately (no interpolation)
  for (int i = 0; i < activeJointCount; i++) {
    if (joints[i].isActive) {
      joints[i].currentAngle = joints[i].targetAngle;
      // Apply calibration offset when writing to hardware
      int hardwareAngle = joints[i].currentAngle + joints[i].calibrationOffset;
      joints[i].servo->write(hardwareAngle);
    }
  }
  moving = false;
}

bool MotionController::isMoving() {
  return moving;
}

int MotionController::getCurrentAngle(int jointIndex) {
  if (jointIndex >= 0 && jointIndex < activeJointCount && joints[jointIndex].isActive) {
    return joints[jointIndex].currentAngle;
  }
  return -1;
}

unsigned long MotionController::getLastUpdateTime() {
  return lastUpdateTime;
}

unsigned long MotionController::getUpdateCycleTime() {
  return lastCycleTime;
}

bool MotionController::isWithinSoftLimits(int jointIndex) {
  if (jointIndex >= 0 && jointIndex < activeJointCount && joints[jointIndex].isActive) {
    int angle = joints[jointIndex].currentAngle;
    return (angle >= joints[jointIndex].softMinAngle && 
            angle <= joints[jointIndex].softMaxAngle);
  }
  return false;
}

// Private helper functions

int MotionController::constrainAngle(int jointIndex, int angle) {
  if (jointIndex < 0 || jointIndex >= activeJointCount || !joints[jointIndex].isActive) {
    return angle;
  }
  
  // Enforce hard limits (mechanical protection)
  if (angle < joints[jointIndex].minAngle) {
    return joints[jointIndex].minAngle;
  }
  if (angle > joints[jointIndex].maxAngle) {
    return joints[jointIndex].maxAngle;
  }
  return angle;
}

int MotionController::applySoftLimits(int jointIndex, int angle) {
  if (jointIndex < 0 || jointIndex >= activeJointCount || !joints[jointIndex].isActive) {
    return angle;
  }
  
  // Apply soft limits for normal operation
  if (angle < joints[jointIndex].softMinAngle) {
    return joints[jointIndex].softMinAngle;
  }
  if (angle > joints[jointIndex].softMaxAngle) {
    return joints[jointIndex].softMaxAngle;
  }
  return angle;
}

void MotionController::updateJoint(int jointIndex) {
  JointConfig* joint = &joints[jointIndex];
  
  if (joint->currentAngle == joint->targetAngle) {
    return; // already at target
  }
  
  // Calculate step direction and magnitude
  int delta = joint->targetAngle - joint->currentAngle;
  int step;
  
  if (abs(delta) <= interpolationSpeed) {
    // Close enough - move directly to target
    step = delta;
  } else {
    // Move incrementally toward target
    step = (delta > 0) ? interpolationSpeed : -interpolationSpeed;
  }
  
  // Enforce velocity limits (safety constraint)
  enforceVelocityLimits(jointIndex, &step);
  
  // Update current angle
  joint->currentAngle += step;
  
  // Ensure we don't overshoot due to rounding
  if ((step > 0 && joint->currentAngle > joint->targetAngle) ||
      (step < 0 && joint->currentAngle < joint->targetAngle)) {
    joint->currentAngle = joint->targetAngle;
  }
  
  // Apply calibration and write to servo hardware
  int hardwareAngle = joint->currentAngle + joint->calibrationOffset;
  hardwareAngle = constrain(hardwareAngle, 0, 180); // Servo library range
  joint->servo->write(hardwareAngle);
}

bool MotionController::allJointsAtTarget() {
  for (int i = 0; i < activeJointCount; i++) {
    if (joints[i].isActive && joints[i].currentAngle != joints[i].targetAngle) {
      return false;
    }
  }
  return true;
}

void MotionController::enforceVelocityLimits(int jointIndex, int* step) {
  // This is a simplified velocity limit check
  // In a real-time system, this would use actual time delta between updates
  // For now, we assume a nominal update rate and scale accordingly
  
  // Assuming 20ms update cycle (50Hz), velocity limit in degrees/sec
  // Max step = (maxVelocity * 20) / 1000
  JointConfig* joint = &joints[jointIndex];
  int maxStep = (joint->maxVelocity * 20) / 1000;
  
  if (abs(*step) > maxStep) {
    *step = (*step > 0) ? maxStep : -maxStep;
  }
}
