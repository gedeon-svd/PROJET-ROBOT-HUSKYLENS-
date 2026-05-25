#include <HUSKYLENS.h>
#include <Wire.h>

// ─── PINS MOTEURS ───────────────────────────────────────────
const int speedPin_M1     = 5;  // Moteur Droit
const int speedPin_M2     = 6;  // Moteur Gauche
const int directionPin_M1 = 4;  // Direction Droit
const int directionPin_M2 = 7;  // Direction Gauche

HUSKYLENS huskylens;

// ─── RÉGLAGES SIMPLES ───────────────────────────────────────
const int CENTER    = 160;
const int DEAD_ZONE = 50;  // Tolérance pour aller tout droit

const int SPEED_STRAIGHT = 110;  
const int SPEED_TURN_MAX = 130;
const int SPEED_TURN_MIN = 15;  

void setup() {
  Serial.begin(115200);
  Wire.begin();

  while (!huskylens.begin(Wire)) {
    Serial.println(F("HuskyLens non détecté..."));
    delay(500);
  }

  Serial.println(F("HuskyLens Prête"));
}

void loop() {
  if (!huskylens.request()) {
    motorStop();
    return;
  }

  bool targetFound = false;
  int objectX = CENTER;

  // On vérifie ce que voit la caméra
  while (huskylens.available()) {
    HUSKYLENSResult result = huskylens.read();
   
    // Si elle voit un bloc ou une flèche qui correspond à ton apprentissage (ID1)
    if (result.command == COMMAND_RETURN_BLOCK || result.command == COMMAND_RETURN_ARROW) {
     
      // S'assurer qu'on ne prend pas en compte les faux reflets en vérifiant l'ID
      if (result.ID == 1) {
        if (result.command == COMMAND_RETURN_BLOCK) {
          objectX = result.xCenter;
        } else {
          objectX = result.xTarget;
        }
        targetFound = true; // L'objet ID1 est bien visible
      }
    }
  }

  // ─── LOGIQUE DE DÉCISION  ──────────────────────────
  if (targetFound) {
    int error = objectX - CENTER;

    // Cas 1 : Bien centré -> En avant tout droit
    if (abs(error) <= DEAD_ZONE) {
      motorForward(SPEED_STRAIGHT, SPEED_STRAIGHT);
    }
    // Cas 2 : L'objet part à gauche -> On tourne à gauche
    else if (error < 0) {
      motorForward(SPEED_TURN_MIN, SPEED_TURN_MAX);
    }
    // Cas 3 : L'objet part à droite -> On tourne à droite
    else {
      motorForward(SPEED_TURN_MAX, SPEED_TURN_MIN);
    }
  }
  // Cas 4 : SI ON ENLÈVE L'OBJET -> ARRÊT IMMEDIAT NÉCESSAIRE
  else {
    motorStop(); // On coupe obligatoirement les moteurs ici
  }
}

// ─── PILOTAGE DES MOTEURS ────────────────────────────────────

void motorStop() {
  analogWrite(speedPin_M2, 0);
  digitalWrite(directionPin_M1, LOW);
  analogWrite(speedPin_M1, 0);
  digitalWrite(directionPin_M2, LOW);
}

void motorForward(int leftSpeed, int rightSpeed) {
  analogWrite(speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, HIGH);
  analogWrite(speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, LOW);
}
