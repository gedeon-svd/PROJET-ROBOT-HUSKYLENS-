#include <HUSKYLENS.h>
#include <Wire.h>
#include <Metro.h>

// ─── VITESSES AJUSTÉES POUR PLUS DE PUISSANCE ET DE STABILITÉ ───
const int speedForward  = 110; // Vitesse dynamique en ligne droite
const int speedTurn     = 180; // Force nécessaire pour tourner la roue extérieure
const int speedSlow     = 60;  // Empêche le robot de caler dans les virages serrés
const int speedBack     = 120; // Recul efficace sans faire de toupie

// ─── PINS MOTEURS ───────────────────────────────────────────
const int speedPin_M1     = 5;
const int speedPin_M2     = 6;
const int directionPin_M1 = 4;
const int directionPin_M2 = 7;

// ─── HUSKYLENS ──────────────────────────────────────────────
HUSKYLENS huskylens;

const int CENTER    = 160;
const int DEAD_ZONE = 45;  // Réduit à 45 pour une précision millimétrique

// ─── TIMERS ─────────────────────────────────────────────────
Metro huskylensTimer = Metro(50);  
Metro debugTimer     = Metro(500);  

enum State {
  FOLLOW,
  ADJUST_LEFT,
  ADJUST_RIGHT,
  LOST 
};

State currentState = LOST;

int  lineX     = CENTER;
int  targetX   = CENTER;
bool lineFound = false;
int  lastDirection = 1; 

void setup() {
  Serial.begin(115200);
  Wire.begin();

  while (!huskylens.begin(Wire)) {
    Serial.println(F("HuskyLens non détecté..."));
    delay(500);
  }

  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
  Serial.println(F("HuskyLens OK - Mode Line Tracking"));
  delay(500);
}

void loop() {
  if (huskylensTimer.check()) {
    readHuskyLens();

    State newState = computeState();

    if (newState != currentState) {
      currentState = newState;
      applyState(currentState);
    }
  }

  if (debugTimer.check()) {
    Serial.print(F("State: "));
    Serial.print(stateLabel(currentState));
    Serial.print(F("  lineX: "));
    Serial.print(lineX);
    Serial.print(F("  lineFound: "));
    Serial.println(lineFound ? "OUI" : "NON");
  }
}

void readHuskyLens() {
  if (!huskylens.request()) {
    lineFound = false;
    return;
  }

  bool currentFound = false;
  while (huskylens.available()) {
    HUSKYLENSResult result = huskylens.read();

    if (result.command == COMMAND_RETURN_ARROW) {
      lineX     = result.xOrigin;
      targetX   = result.xTarget;
      currentFound = true;
     
      if (targetX < CENTER) {
        lastDirection = -1; 
      } else if (targetX > CENTER) {
        lastDirection = 1;  
      }
    }
  }
  lineFound = currentFound;
}

State computeState() {
  if (!lineFound) {
    return LOST; 
  }

  int error = lineX - CENTER;
  int directionError = targetX - CENTER;

  if (abs(directionError) > DEAD_ZONE) {
    if (directionError < 0) return ADJUST_LEFT;
    else return ADJUST_RIGHT;
  }

  if (error < -DEAD_ZONE) return ADJUST_LEFT;
  if (error >  DEAD_ZONE) return ADJUST_RIGHT;
 
  return FOLLOW;
}

void applyState(State s) {
  switch (s) {
    case FOLLOW:
      carAdvance(speedForward, speedForward);
      break;
     
    case ADJUST_LEFT:
      carAdjust(speedSlow, speedTurn);
      break;
     
    case ADJUST_RIGHT:
      carAdjust(speedTurn, speedSlow);
      break;
     
    case LOST:
      if (lastDirection == 1) {
        carBack(speedSlow, speedBack);
      } else {
        carBack(speedBack, speedSlow);
      }
      break;
  }
}

const char* stateLabel(State s) {
  switch (s) {
    case FOLLOW:       return "FOLLOW";
    case ADJUST_LEFT:  return "ADJUST_LEFT";
    case ADJUST_RIGHT: return "ADJUST_RIGHT";
    case LOST:         return "LOST";
    default:           return "?";
  }
}

void carStop() {
  analogWrite(speedPin_M2, 0);
  digitalWrite(directionPin_M1, LOW);
  analogWrite(speedPin_M1, 0);
  digitalWrite(directionPin_M2, LOW);
}

void carAdvance(int leftSpeed, int rightSpeed) {
  analogWrite(speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, HIGH);
  analogWrite(speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, LOW);
}

void carAdjust(int leftSpeed, int rightSpeed) {
  analogWrite(speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, HIGH);
  analogWrite(speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, LOW);
}

void carBack(int leftSpeed, int rightSpeed){         
  analogWrite (speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, LOW);
  analogWrite (speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, HIGH);
}
