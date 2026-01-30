/*
 * Robotic Arm Control - Enhanced Main Sketch
 * Hardware: Arduino UNO/Nano, 3-4 servos (SG90/MG995)
 * External 5V power supply required for servos (shared ground with Arduino)
 * 
 * Features:
 * - Coordinated multi-joint motion with interpolation
 * - Safety limits and emergency stop
 * - Serial command interface for external control
 * - Timing diagnostics for control loop analysis
 * - Calibration support
 */

#include <Servo.h>
#include "motion_controller.h"

// Servo objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

// Pin assignments
const int BASE_PIN = 9;
const int SHOULDER_PIN = 10;
const int ELBOW_PIN = 11;
const int GRIPPER_PIN = 6;
const int EMERGENCY_STOP_PIN = 2; // Optional physical E-stop button

// Motion controller instance
MotionController motionCtrl;

// Joint-specific limits (adjust for your mechanical design)
const int BASE_MIN = 0;
const int BASE_MAX = 180;
const int SHOULDER_MIN = 15;   // Prevent collision with base
const int SHOULDER_MAX = 165;
const int ELBOW_MIN = 0;
const int ELBOW_MAX = 180;
const int GRIPPER_MIN = 10;    // Prevent over-closing
const int GRIPPER_MAX = 90;    // Prevent over-opening

// Predefined poses (joint angles in degrees)
const int HOME_POSE[NUM_JOINTS] = {90, 90, 90, 45};
const int TARGET_POSE_1[NUM_JOINTS] = {45, 60, 120, 45};
const int TARGET_POSE_2[NUM_JOINTS] = {135, 110, 70, 70};
const int TARGET_POSE_3[NUM_JOINTS] = {90, 45, 135, 45};

// Calibration offsets (set these after physical calibration)
const int CALIBRATION_OFFSETS[NUM_JOINTS] = {0, 0, 0, 0};

// Sequence control
enum SequenceState {
  SEQ_IDLE,
  SEQ_HOME,
  SEQ_MOVE_TO_TARGET_1,
  SEQ_MOVE_TO_TARGET_2,
  SEQ_MOVE_TO_TARGET_3,
  SEQ_RETURN_HOME,
  SEQ_PAUSE,
  SEQ_COMPLETE
};

SequenceState currentState = SEQ_IDLE;
unsigned long pauseStartTime = 0;
const unsigned long PAUSE_DURATION = 1000; // ms

// Timing and diagnostics
unsigned long loopCount = 0;
unsigned long lastDiagnosticTime = 0;
const unsigned long DIAGNOSTIC_INTERVAL = 5000; // 5 seconds

// Serial command buffer
String serialBuffer = "";
bool autoSequenceEnabled = false;

void setup() {
  Serial.begin(115200); // Higher baud for better responsiveness
  Serial.println(F("==================================="));
  Serial.println(F("Robotic Arm Control System"));
  Serial.println(F("Enhanced Motion Controller v2.0"));
  Serial.println(F("==================================="));
  
  // Configure emergency stop pin if used
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  
  // Attach servos to pins
  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  gripperServo.attach(GRIPPER_PIN);
  
  // Register servos with motion controller
  motionCtrl.addServo(&baseServo, BASE_MIN, BASE_MAX);
  motionCtrl.addServo(&shoulderServo, SHOULDER_MIN, SHOULDER_MAX);
  motionCtrl.addServo(&elbowServo, ELBOW_MIN, ELBOW_MAX);
  motionCtrl.addServo(&gripperServo, GRIPPER_MIN, GRIPPER_MAX);
  
  // Apply calibration offsets
  for (int i = 0; i < NUM_JOINTS; i++) {
    motionCtrl.setCalibrationOffset(i, CALIBRATION_OFFSETS[i]);
  }
  
  // Set velocity limits (degrees per second)
  motionCtrl.setMaxVelocity(0, 60);  // Base: slower for stability
  motionCtrl.setMaxVelocity(1, 45);  // Shoulder: slower due to load
  motionCtrl.setMaxVelocity(2, 60);  // Elbow: moderate speed
  motionCtrl.setMaxVelocity(3, 90);  // Gripper: faster (low inertia)
  
  // Validate home pose before moving
  if (!motionCtrl.validatePose(HOME_POSE)) {
    Serial.println(F("ERROR: Home pose outside safe limits!"));
    Serial.println(F("System halted. Check joint limit configuration."));
    while(1); // Halt execution
  }
  
  // Initialize to home position slowly
  Serial.println(F("Moving to home position..."));
  motionCtrl.setTargetAngles(HOME_POSE);
  motionCtrl.setInterpolationSpeed(1); // Slow initial move
  
  // Wait for home position to complete
  while (motionCtrl.isMoving()) {
    motionCtrl.update();
    delay(20);
  }
  
  Serial.println(F("Initialization complete."));
  Serial.println(F(""));
  printHelp();
}

void loop() {
  // Check hardware emergency stop button
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    if (!motionCtrl.isEmergencyStopped()) {
      motionCtrl.emergencyStop();
      Serial.println(F("EMERGENCY STOP ACTIVATED!"));
      currentState = SEQ_IDLE;
      autoSequenceEnabled = false;
    }
  }
  
  // Process serial commands
  processSerialCommands();
  
  // Update motion controller (must be called regularly)
  motionCtrl.update();
  
  // Sequence state machine (only if auto sequence enabled)
  if (autoSequenceEnabled) {
    runSequenceStateMachine();
  }
  
  // Periodic diagnostics
  loopCount++;
  if (millis() - lastDiagnosticTime >= DIAGNOSTIC_INTERVAL) {
    printDiagnostics();
    lastDiagnosticTime = millis();
  }
  
  // Control loop timing: 20ms = 50Hz update rate
  // Critical for consistent motion and timing determinism
  delay(20);
}

void runSequenceStateMachine() {
  switch (currentState) {
    case SEQ_IDLE:
      // Waiting for command
      break;
      
    case SEQ_HOME:
      if (!motionCtrl.isMoving()) {
        Serial.println(F("State: HOME -> Moving to Target 1"));
        motionCtrl.setTargetAngles(TARGET_POSE_1);
        motionCtrl.setInterpolationSpeed(2);
        currentState = SEQ_MOVE_TO_TARGET_1;
      }
      break;
      
    case SEQ_MOVE_TO_TARGET_1:
      if (!motionCtrl.isMoving()) {
        Serial.println(F("State: TARGET 1 -> Pausing"));
        pauseStartTime = millis();
        currentState = SEQ_PAUSE;
      }
      break;
      
    case SEQ_PAUSE:
      if (millis() - pauseStartTime >= PAUSE_DURATION) {
        Serial.println(F("State: PAUSE -> Moving to Target 2"));
        motionCtrl.setTargetAngles(TARGET_POSE_2);
        motionCtrl.setInterpolationSpeed(2);
        currentState = SEQ_MOVE_TO_TARGET_2;
      }
      break;
      
    case SEQ_MOVE_TO_TARGET_2:
      if (!motionCtrl.isMoving()) {
        Serial.println(F("State: TARGET 2 -> Moving to Target 3"));
        motionCtrl.setTargetAngles(TARGET_POSE_3);
        motionCtrl.setInterpolationSpeed(3);
        currentState = SEQ_MOVE_TO_TARGET_3;
      }
      break;
      
    case SEQ_MOVE_TO_TARGET_3:
      if (!motionCtrl.isMoving()) {
        Serial.println(F("State: TARGET 3 -> Returning home"));
        motionCtrl.setTargetAngles(HOME_POSE);
        motionCtrl.setInterpolationSpeed(2);
        currentState = SEQ_RETURN_HOME;
      }
      break;
      
    case SEQ_RETURN_HOME:
      if (!motionCtrl.isMoving()) {
        Serial.println(F("State: HOME -> Sequence complete"));
        currentState = SEQ_COMPLETE;
      }
      break;
      
    case SEQ_COMPLETE:
      Serial.println(F("Sequence finished. Waiting for next command."));
      autoSequenceEnabled = false;
      currentState = SEQ_IDLE;
      break;
  }
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd == "HELP" || cmd == "?") {
    printHelp();
  }
  else if (cmd == "START") {
    if (motionCtrl.isEmergencyStopped()) {
      Serial.println(F("Cannot start: Emergency stop active. Send RESET first."));
    } else {
      Serial.println(F("Starting automatic sequence..."));
      autoSequenceEnabled = true;
      currentState = SEQ_HOME;
    }
  }
  else if (cmd == "STOP") {
    Serial.println(F("Stopping sequence..."));
    autoSequenceEnabled = false;
    currentState = SEQ_IDLE;
  }
  else if (cmd == "ESTOP") {
    motionCtrl.emergencyStop();
    autoSequenceEnabled = false;
    currentState = SEQ_IDLE;
    Serial.println(F("EMERGENCY STOP ACTIVATED!"));
  }
  else if (cmd == "RESET") {
    motionCtrl.resetEmergencyStop();
    Serial.println(F("Emergency stop reset. System ready."));
  }
  else if (cmd == "HOME") {
    if (!motionCtrl.isEmergencyStopped()) {
      Serial.println(F("Moving to home position..."));
      motionCtrl.setTargetAngles(HOME_POSE);
      motionCtrl.setInterpolationSpeed(2);
    }
  }
  else if (cmd == "STATUS") {
    printStatus();
  }
  else if (cmd == "DIAG") {
    printDiagnostics();
  }
  else {
    Serial.println(F("Unknown command. Type HELP for available commands."));
  }
}

void printHelp() {
  Serial.println(F("Available Commands:"));
  Serial.println(F("  START  - Start automatic motion sequence"));
  Serial.println(F("  STOP   - Stop automatic sequence"));
  Serial.println(F("  HOME   - Return to home position"));
  Serial.println(F("  ESTOP  - Emergency stop (halt all motion)"));
  Serial.println(F("  RESET  - Reset emergency stop"));
  Serial.println(F("  STATUS - Print current joint angles"));
  Serial.println(F("  DIAG   - Print diagnostics"));
  Serial.println(F("  HELP   - Show this help message"));
  Serial.println(F(""));
}

void printStatus() {
  Serial.println(F("--- System Status ---"));
  Serial.print(F("Emergency Stop: "));
  Serial.println(motionCtrl.isEmergencyStopped() ? F("ACTIVE") : F("Inactive"));
  Serial.print(F("Moving: "));
  Serial.println(motionCtrl.isMoving() ? F("Yes") : F("No"));
  Serial.println(F("Joint Angles (degrees):"));
  for (int i = 0; i < NUM_JOINTS; i++) {
    Serial.print(F("  Joint "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(motionCtrl.getCurrentAngle(i));
    Serial.print(F(" [Soft limits: "));
    Serial.print(motionCtrl.isWithinSoftLimits(i) ? F("OK") : F("WARN"));
    Serial.println(F("]"));
  }
  Serial.println(F(""));
}

void printDiagnostics() {
  Serial.println(F("--- Diagnostics ---"));
  Serial.print(F("Loop count: "));
  Serial.println(loopCount);
  Serial.print(F("Avg loop rate: "));
  Serial.print((loopCount * 1000.0) / millis());
  Serial.println(F(" Hz"));
  Serial.print(F("Last update cycle: "));
  Serial.print(motionCtrl.getUpdateCycleTime());
  Serial.println(F(" us"));
  Serial.print(F("Free RAM: "));
  Serial.print(freeRam());
  Serial.println(F(" bytes"));
  Serial.println(F(""));
}

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
