/*
 * Communication: 115200 baud, newline-terminated commands
 */

// ============== PIN DEFINITIONS ==============

// Motor Control (YL12 BLDC Controllers)
const int LEFT_REVERSE_PIN = 4;   
const int LEFT_BRAKE_PIN   = 5;   
const int RIGHT_REVERSE_PIN = 6; 
const int RIGHT_BRAKE_PIN   = 7;

// Vacuum System
const int VACUUM_RELAY_PIN = 8;   

// Robotic Arm (Servo or Relay)
const int ARM_SERVO_PIN = 9;      
const int ARM_GRIPPER_PIN = 10;  

// Wiper System
const int WIPER_MOTOR_PIN = 11;   

// UV Sanitization Strip
const int UV_STRIP_PIN = 12;     

// Emergency LED indicator
const int STATUS_LED_PIN = 13;    

// ============== SYSTEM STATE ==============

struct SystemState {
  bool vacuum_active;
  bool arm_active;
  bool wiper_active;
  bool uv_active;
  bool autonomous_mode;
  bool emergency_stop;
  bool moving;
} state;

// Command buffer
String commandBuffer = "";
const int BUFFER_SIZE = 64;

// Timing for non-blocking operations
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 1000; 

// ============== SETUP ==============

void setup() {
  // Initialize motor control pins
  pinMode(LEFT_REVERSE_PIN, OUTPUT);
  pinMode(LEFT_BRAKE_PIN, OUTPUT);
  pinMode(RIGHT_REVERSE_PIN, OUTPUT);
  pinMode(RIGHT_BRAKE_PIN, OUTPUT);
  
  // Initialize component control pins
  pinMode(VACUUM_RELAY_PIN, OUTPUT);
  pinMode(ARM_SERVO_PIN, OUTPUT);
  pinMode(ARM_GRIPPER_PIN, OUTPUT);
  pinMode(WIPER_MOTOR_PIN, OUTPUT);
  pinMode(UV_STRIP_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Set all to safe initial state
  stopAllMotors();
  digitalWrite(VACUUM_RELAY_PIN, LOW);
  digitalWrite(ARM_SERVO_PIN, LOW);
  digitalWrite(ARM_GRIPPER_PIN, LOW);
  digitalWrite(WIPER_MOTOR_PIN, LOW);
  digitalWrite(UV_STRIP_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, HIGH);
  
  // Initialize state
  state.vacuum_active = false;
  state.arm_active = false;
  state.wiper_active = false;
  state.uv_active = false;
  state.autonomous_mode = false;
  state.emergency_stop = false;
  state.moving = false;
  
  // Start serial communication at 115200 baud
  Serial.begin(115200);
  Serial.println("SWACCH:READY");
  Serial.println("Automated Vacuum Cleaner Controller v1.0");
  Serial.println("Waiting for HMI commands...");
  
  // Blink LED to indicate ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
  }
}

// ============== MAIN LOOP ==============

void loop() {
  // Process incoming serial commands
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      if (commandBuffer.length() > 0) {
        processCommand(commandBuffer);
        commandBuffer = "";
      }
    } else if (commandBuffer.length() < BUFFER_SIZE) {
      commandBuffer += inChar;
    }
  }
  
  // Periodic status updates
  if (millis() - lastStatusUpdate > STATUS_INTERVAL) {
    sendStatusUpdate();
    lastStatusUpdate = millis();
  }
  
  // Emergency stop LED blink
  if (state.emergency_stop) {
    digitalWrite(STATUS_LED_PIN, (millis() / 250) % 2);
  } else {
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
}

// ============== COMMAND PROCESSING ==============

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  Serial.print("CMD_RECV: ");
  Serial.println(cmd);
  
  // Emergency stop - highest priority
  if (cmd == "ESTOP") {
    emergencyStop();
    return;
  }
  
  // Reset from emergency stop
  if (cmd == "RESET") {
    resetSystem();
    return;
  }
  
  // Don't process other commands
  if (state.emergency_stop) {
    Serial.println("ERR:EMERGENCY_STOP_ACTIVE");
    return;
  }
  
  // Vacuum control
  if (cmd == "VACUUM:ON") {
    activateVacuum(true);
  } else if (cmd == "VACUUM:OFF") {
    activateVacuum(false);
  }
  
  // Arm control
  else if (cmd == "ARM:ON") {
    activateArm(true);
  } else if (cmd == "ARM:OFF") {
    activateArm(false);
  }
  
  // Wiper control
  else if (cmd == "WIPER:ON") {
    activateWiper(true);
  } else if (cmd == "WIPER:OFF") {
    activateWiper(false);
  }
  
  // UV control
  else if (cmd == "UV:ON") {
    activateUV(true);
  } else if (cmd == "UV:OFF") {
    activateUV(false);
  }
  
  // Autonomous mode
  else if (cmd == "AUTO:ON") {
    state.autonomous_mode = true;
    Serial.println("ACK:AUTO_MODE_ON");
  } else if (cmd == "AUTO:OFF") {
    state.autonomous_mode = false;
    stopAllMotors();
    Serial.println("ACK:AUTO_MODE_OFF");
  }
  
  // AI detection (placeholder)
  else if (cmd == "AI:ON") {
    Serial.println("ACK:AI_DETECTION_ON");
  } else if (cmd == "AI:OFF") {
    Serial.println("ACK:AI_DETECTION_OFF");
  }
  
  // Movement controls (manual mode)
  else if (cmd == "MOVE:FORWARD") {
    if (!state.autonomous_mode) moveForward();
  } else if (cmd == "MOVE:BACKWARD") {
    if (!state.autonomous_mode) moveBackward();
  } else if (cmd == "MOVE:LEFT") {
    if (!state.autonomous_mode) turnLeft();
  } else if (cmd == "MOVE:RIGHT") {
    if (!state.autonomous_mode) turnRight();
  } else if (cmd == "MOVE:STOP") {
    if (!state.autonomous_mode) stopAllMotors();
  }
  
  // Unknown command
  else {
    Serial.print("ERR:UNKNOWN_CMD:");
    Serial.println(cmd);
  }
}

// ============== COMPONENT CONTROL ==============

void activateVacuum(bool active) {
  state.vacuum_active = active;
  digitalWrite(VACUUM_RELAY_PIN, active ? HIGH : LOW);
  Serial.print("ACK:VACUUM_");
  Serial.println(active ? "ON" : "OFF");
}

void activateArm(bool active) {
  state.arm_active = active;
  // Might want to use Servo library
  digitalWrite(ARM_SERVO_PIN, active ? HIGH : LOW);
  Serial.print("ACK:ARM_");
  Serial.println(active ? "ON" : "OFF");
}

void activateWiper(bool active) {
  state.wiper_active = active;
  digitalWrite(WIPER_MOTOR_PIN, active ? HIGH : LOW);
  Serial.print("ACK:WIPER_");
  Serial.println(active ? "ON" : "OFF");
}

void activateUV(bool active) {
  state.uv_active = active;
  digitalWrite(UV_STRIP_PIN, active ? HIGH : LOW);
  Serial.print("ACK:UV_");
  Serial.println(active ? "ON" : "OFF");
}

// ============== MOTOR CONTROL ==============

void moveForward() {
  digitalWrite(LEFT_BRAKE_PIN, HIGH);
  digitalWrite(RIGHT_BRAKE_PIN, HIGH);
  digitalWrite(LEFT_REVERSE_PIN, LOW);
  digitalWrite(RIGHT_REVERSE_PIN, LOW);
  state.moving = true;
  Serial.println("ACK:MOVE_FORWARD");
}

void moveBackward() {
  digitalWrite(LEFT_BRAKE_PIN, HIGH);
  digitalWrite(RIGHT_BRAKE_PIN, HIGH);
  digitalWrite(LEFT_REVERSE_PIN, HIGH);
  digitalWrite(RIGHT_REVERSE_PIN, HIGH);
  state.moving = true;
  Serial.println("ACK:MOVE_BACKWARD");
}

void turnLeft() {
  // Stop left motor, run right motor
  digitalWrite(LEFT_BRAKE_PIN, LOW);
  digitalWrite(RIGHT_BRAKE_PIN, HIGH);
  digitalWrite(RIGHT_REVERSE_PIN, LOW);
  state.moving = true;
  Serial.println("ACK:TURN_LEFT");
}

void turnRight() {
  // Stop right motor, run left motor
  digitalWrite(RIGHT_BRAKE_PIN, LOW);
  digitalWrite(LEFT_BRAKE_PIN, HIGH);
  digitalWrite(LEFT_REVERSE_PIN, LOW);
  state.moving = true;
  Serial.println("ACK:TURN_RIGHT");
}

void stopAllMotors() {
  digitalWrite(LEFT_BRAKE_PIN, LOW);
  digitalWrite(RIGHT_BRAKE_PIN, LOW);
  state.moving = false;
  Serial.println("ACK:MOTORS_STOPPED");
}

// ============== SAFETY FUNCTIONS ==============

void emergencyStop() {
  state.emergency_stop = true;
  
  // Stop all motors
  stopAllMotors();
  
  // Deactivate all systems
  activateVacuum(false);
  activateArm(false);
  activateWiper(false);
  activateUV(false);
  
  state.autonomous_mode = false;
  
  Serial.println("ACK:EMERGENCY_STOP");
  Serial.println("WARN:ALL_SYSTEMS_HALTED");
}

void resetSystem() {
  state.emergency_stop = false;
  Serial.println("ACK:SYSTEM_RESET");
  Serial.println("STATUS:READY");
}

// ============== STATUS REPORTING ==============

void sendStatusUpdate() {
  // Send periodic status to HMI (optional, for monitoring)
  // Format: STATUS:field1=value1,field2=value2,...
  Serial.print("STATUS:");
  Serial.print("VAC=");
  Serial.print(state.vacuum_active ? "1" : "0");
  Serial.print(",ARM=");
  Serial.print(state.arm_active ? "1" : "0");
  Serial.print(",WPR=");
  Serial.print(state.wiper_active ? "1" : "0");
  Serial.print(",UV=");
  Serial.print(state.uv_active ? "1" : "0");
  Serial.print(",AUTO=");
  Serial.print(state.autonomous_mode ? "1" : "0");
  Serial.print(",ESTOP=");
  Serial.print(state.emergency_stop ? "1" : "0");
  Serial.print(",MOVE=");
  Serial.println(state.moving ? "1" : "0");
}
