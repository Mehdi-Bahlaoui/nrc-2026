#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <NewPing.h> 

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// --- HARDWARE DEFS ---
#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 200 // cm
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

const int SERVO_COUNT = 9; // 6 legs + 3 arm
const int NUM_CALIB_POINTS = 7; 
const float CALIB_STEP = 30.0;

// THE THEORETICAL 10-POINT CALIBRATION MAP
// Assuming linear servos. *WARNING:* You will likely need to tweak the 90-degree 
// column (Index 4 & 5 interpolation) if the robot physically leans when commanded to 90.
// Angles:          0,   20,   40,   60,   80,  100,  120,  140,  160,  180
uint16_t pwmMap[SERVO_COUNT][NUM_CALIB_POINTS] = {
  {500,  722,  944, 1166, 1388, 1611, 1833}, // S0: Right Hip
  {500,  722,  944, 1166, 1388, 1611, 1833}, // S1: Right Knee
  {500,  722,  944, 1166, 1388, 1611, 1833}, // S2: Right Ankle
  {500,  722,  944, 1166, 1388, 1611, 1833}, // S3: Left Hip
  {500,  722,  944, 1166, 1388, 1611, 1833}, // S4: Left Knee
  {500,  722,  944, 1166, 1388, 1611, 1833}, // S5: Left Ankle
  {500,  722,  944, 1166, 1388, 1611, 1833}, // S6: Arm Base
  {500,  722,  944, 1166, 1388, 1611, 1833}, // S7: Arm Shoulder
  {500,  722,  944, 1166, 1388, 1611, 1833}  // S8: Arm Elbow/Claw
};

// --- SYSTEM STATE ---
float currentAngles[SERVO_COUNT];
float startAngles[SERVO_COUNT];
float targetAngles[SERVO_COUNT];

unsigned long animStartTime = 0;
int animDuration = 0;
bool isAnimating = false;

int heightOffset = 0; // Dynamic Z-Axis
bool isArmStowed = true;

// The Complete 8-Step Gait
enum WalkState { IDLE, LEAN_R, LIFT_L, STEP_L, LOWER_L, LEAN_L, LIFT_R, STEP_R, LOWER_R };
WalkState currentState = IDLE;

// --- KINEMATICS ---
uint16_t angleToPWM(int servoIndex, float targetAngle) { 
  targetAngle = constrain(targetAngle, 0, 180);
  int lowerIndex = floor(targetAngle / CALIB_STEP);
  int upperIndex = min(lowerIndex + 1, NUM_CALIB_POINTS - 1);
  float remainder = targetAngle - (lowerIndex * CALIB_STEP);
  float weight = remainder / CALIB_STEP;
  uint16_t lowerPWM = pwmMap[servoIndex][lowerIndex];
  uint16_t upperPWM = pwmMap[servoIndex][upperIndex];
  return lowerPWM + (upperPWM - lowerPWM) * weight;
}

void applyPose() {
  for(int i=0; i<SERVO_COUNT; i++) {
    pca.setPWM(i, 0, (int)((angleToPWM(i, currentAngles[i]) * 4096.0) / 20000.0));
  }
}

float easeInOut(float t) { return t * t * (3.0 - 2.0 * t); }

void getBasePose(float t[]) {
  // Legs (Assume Symmetrically Mounted. If one leg moves backward when it should 
  // move forward, invert the '+' to a '-' for that specific joint here).
  t[0] = 90 + heightOffset;         // R_Hip
  t[1] = 90 - (heightOffset * 2);   // R_Knee
  t[2] = 90 + heightOffset;         // R_Ankle
  
  t[3] = 90 - heightOffset;         // L_Hip
  t[4] = 90 + (heightOffset * 2);   // L_Knee
  t[5] = 90 - heightOffset;         // L_Ankle
  
  // Arm Positions
  if(isArmStowed) {
    t[6] = 90; t[7] = 20; t[8] = 160; // Tucked tight for Center of Gravity
  } else {
    t[6] = 90; t[7] = 90; t[8] = 90;  // Extended out to grab Labyrinth Key
  }
}

// --- ANIMATION & FSM ---
void triggerAnimation(float targets[], int durationMs) {
  for(int i=0; i<SERVO_COUNT; i++) { 
    startAngles[i] = currentAngles[i]; 
    targetAngles[i] = targets[i]; 
  }
  animDuration = durationMs; 
  animStartTime = millis(); 
  isAnimating = true;
}

void executeEmergencyStop() {
  float safePose[SERVO_COUNT];
  getBasePose(safePose);
  triggerAnimation(safePose, 200); // 200ms snap to balanced base pose
  currentState = IDLE;
}

void advanceWalkCycle() {
  if (currentState == IDLE) return;
  float t[SERVO_COUNT];
  getBasePose(t); // Load foundation (including squat height)
  
  int speed = 250; // Milliseconds per gait frame (tune this for stability)
  
  switch(currentState) {
    case LEAN_R: 
      t[0] += 25; t[3] -= 25; // Hips tilt to shift CoG over Right Foot
      triggerAnimation(t, speed); 
      currentState = LIFT_L; 
      break;
      
    case LIFT_L: 
      t[0] += 25; t[3] -= 25; // Hold Lean
      t[4] += 35; t[5] -= 15; // Left Knee bends up, Ankle levels the foot
      triggerAnimation(t, speed); 
      currentState = STEP_L; 
      break;
      
    case STEP_L: 
      t[0] += 25; t[3] += 15; // Hold Lean, Left Hip swings forward
      t[4] += 35; t[5] -= 15; // Hold Lift
      triggerAnimation(t, speed); 
      currentState = LOWER_L; 
      break;
      
    case LOWER_L: 
      t[3] += 15; // Hold Forward Step
      // Knees and Ankles return to base to plant the foot
      triggerAnimation(t, speed); 
      currentState = LEAN_L; 
      break;

    case LEAN_L: 
      t[0] -= 25; t[3] += 25; // Shift CoG over the newly planted Left Foot
      triggerAnimation(t, speed); 
      currentState = LIFT_R; 
      break;
      
    case LIFT_R: 
      t[0] -= 25; t[3] += 25; // Hold Lean
      t[1] -= 35; t[2] += 15; // Right Knee bends up, Ankle levels
      triggerAnimation(t, speed); 
      currentState = STEP_R; 
      break;
      
    case STEP_R: 
      t[0] -= 15; t[3] += 25; // Right Hip swings forward, Hold Lean
      t[1] -= 35; t[2] += 15; // Hold Lift
      triggerAnimation(t, speed); 
      currentState = LOWER_R; 
      break;
      
    case LOWER_R: 
      t[0] -= 15; // Hold Forward Step
      // Knees and Ankles return to base to plant
      triggerAnimation(t, speed); 
      currentState = LEAN_R; // Loop back to start
      break;
  }
}

void updateAnimation() {
  if (!isAnimating) return;
  unsigned long elapsed = millis() - animStartTime;
  if (elapsed >= animDuration) {
    for (int i=0; i<SERVO_COUNT; i++) currentAngles[i] = targetAngles[i];
    applyPose();
    isAnimating = false;
    advanceWalkCycle();
  } else {
    float k = easeInOut((float)elapsed / animDuration);
    for (int i=0; i<SERVO_COUNT; i++) currentAngles[i] = startAngles[i] + (targetAngles[i] - startAngles[i]) * k;
    applyPose();
  }
}

// --- MAIN LOOP ---
void setup() {
  Serial.begin(115200); 
  pca.begin(); 
  pca.setPWMFreq(50);
  
  // Initialize to safe standing posture
  getBasePose(currentAngles); 
  applyPose(); 
  delay(1000); 
}

void loop() {
  // 1. Keep the gait flowing smoothly
  updateAnimation();
  
  // 2. Telemetry / Teleoperation
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'W' && currentState == IDLE) { 
      currentState = LEAN_R; // Kick off the walk
      advanceWalkCycle(); 
    } 
    else if (cmd == 'S') { 
      executeEmergencyStop(); // Instantly abort and plant feet
    } 
    else if (cmd == 'D') { 
      heightOffset += 5; // Drop Z-Axis
      if(currentState == IDLE) executeEmergencyStop(); 
    } 
    else if (cmd == 'U') { 
      heightOffset -= 5; // Raise Z-Axis
      if(currentState == IDLE) executeEmergencyStop(); 
    } 
    else if (cmd == 'A') { 
      isArmStowed = !isArmStowed; // Deploy/Stow Arm
      if(currentState == IDLE) executeEmergencyStop(); 
    }
  }

  // 3. Collision Braking (Optional)
  static unsigned long lastPing = 0;
  if (millis() - lastPing > 50) {
    lastPing = millis();
    unsigned int dist = sonar.ping_cm();
    // Emergency brake if wall is < 10cm and robot is walking
    if (dist > 0 && dist < 10 && currentState != IDLE) {
      executeEmergencyStop(); 
    }
  }
}