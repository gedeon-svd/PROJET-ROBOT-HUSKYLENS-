# Rapport de Projet : Robot Mobile Autonome avec HuskyLens

Ce dépôt rassemble les différents codes Arduino développés pour notre robot mobile équipé d'une caméra intelligente **HuskyLens**. Ce projet explore la navigation autonome, du labyrinthe au suivi de trajectoire par intelligence artificielle.

## 🤖 Présentation du Matériel
Voici l'apparence générale de notre robot ainsi que sa caméra intelligente HuskyLens montée sur son châssis.

| Vue d'ensemble du Robot | Zoom sur la Caméra HuskyLens |
| :---: | :---: |
| ![Photo du Robot Complet](<img width="974" height="1080" alt="WhatsApp Image 2026-05-25 at 17 34 15" src="https://github.com/user-attachments/assets/3ba34f29-64ab-4ec5-921c-f833b51e811a" />
) | ![Détail de la caméra](<img width="1284" height="1701" alt="WhatsApp Image 2026-05-25 at 17 37 33" src="https://github.com/user-attachments/assets/441e3f6f-9e1f-44ab-9edf-f873c610f0a8" />
) | 
Pour en savoir plus sur le cablage du servo moteur et du capteur ultrason utilisé:https://wiki.dfrobot.com/rob0117/docs/21533 .


--- 
# PROJET-ROBOT-HUSKYLENS- 
## Partie 1 : Sortir du Labyrinthe (Logique et Capteurs)

### Description
Cette première partie consistait à faire naviguer le robot de manière autonome dans un labyrinthe en utilisant un capteur de distance à ultrasons monté sur un servomoteur. Le servomoteur sert de "cou" au robot pour faire pivoter le capteur et regarder à droite et à gauche.

### Fonctionnement et Logique du Robot

1. **En ligne droite (Mode normal) :**
   Tant que la voie est libre (distance mesurée supérieure à 25 cm), le robot avance tout droit à une vitesse modérée. En même temps, le servomoteur fait osciller le capteur en continu de gauche à droite entre 60° et 120° pour anticiper les obstacles.

2. **Face à un mur (Mode évitement) :**
   Dès qu'un obstacle est détecté à 25 cm ou moins, le robot s'arrête, recule pendant 300 ms pour se dégager, puis utilise son servomoteur comme un radar :
   - Il regarde complètement à droite (30°) et enregistre la distance.
   - Il regarde complètement à gauche (150°) et enregistre la distance.
   - Il compare les deux valeurs et pivote pendant 450 ms du côté où le chemin est le plus libre avant de repartir en marche avant.

### Visuels du Labyrinthe
### Code Arduino
```cpp
#include <Servo.h>
#include <Metro.h>

int speedPin_M1 = 5;     //M1 Speed Control
int speedPin_M2 = 6;     //M2 Speed Control
int directionPin_M1 = 4;     //M1 Direction Control
int directionPin_M2 = 7;     //M2 Direction Control
unsigned long actualDistance = 0;

Servo myservo;  
int pos = 90;
int sweepFlag = 1;

int URPWM = 3; 
int URTRIG = 10; 
uint8_t EnPwmCmd[4] = {0x44, 0x02, 0xbb, 0x01}; 

Metro measureDistance = Metro(50);
Metro sweepServo = Metro(20);

void setup(){                                 
  myservo.attach(9);
  Serial.begin(9600);                         
  SensorSetup();
  myservo.write(90);
  delay(1000);
}

void loop() {
  if (measureDistance.check() == 1) {
    actualDistance = MeasureDistance();
    Serial.println(actualDistance);
  }

  if (actualDistance <= 25 && actualDistance > 0) {
    carAdvance(0, 0);
    delay(200);

    carBack(100, 100);
    delay(300);
    carAdvance(0, 0);
    delay(200);

    myservo.write(30);
    delay(600);
    int distDroit = MeasureDistance();

    myservo.write(150);
    delay(600);
    int distGauche = MeasureDistance();

    myservo.write(90);
    delay(300);

    if (distDroit > distGauche + 3) {
      carTurnRight(180, 180);
      delay(450);
    } else {
      carTurnLeft(180, 180);
      delay(450);
    }
   
    carAdvance(0, 0);
    actualDistance = 100;
  } else {
    if(sweepServo.check() == 1) {
      if(sweepFlag == 1) {
        pos += 5;
        if (pos>=120) sweepFlag =0;
      } else {
        pos -= 5;
        if (pos<=60) sweepFlag = 1;  
      }
      myservo.write(pos);
    }
   
    carAdvance(100, 100);
  }
}

void SensorSetup() {
  pinMode(URTRIG, OUTPUT);                    
  digitalWrite(URTRIG, HIGH);                 
  pinMode(URPWM, INPUT);                      
  for (int i = 0; i < 4; i++) {
    Serial.write(EnPwmCmd[i]);
  }
}

int MeasureDistance() { 
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);               
  unsigned long distance = pulseIn(URPWM, LOW);
  if (distance == 1000) {          
    Serial.print("Invalid");
  } else {
    distance = distance / 50;       
  }
  return distance;
}

void carStop() {                
  digitalWrite(speedPin_M2, 0);
  digitalWrite(directionPin_M1, LOW);
  digitalWrite(speedPin_M1, 0);
  digitalWrite(directionPin_M2, LOW);
}

void carAdvance(int leftSpeed, int rightSpeed) {     
  analogWrite (speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, HIGH);
  analogWrite (speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, LOW);
}

void carBack(int leftSpeed, int rightSpeed) {       
  analogWrite (speedPin_M2, leftSpeed);             
  digitalWrite(directionPin_M1, LOW);              
  analogWrite (speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, HIGH);
}

void carTurnLeft(int leftSpeed, int rightSpeed) {           
  analogWrite (speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, HIGH);
  analogWrite (speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, HIGH);
}

void carTurnRight(int leftSpeed, int rightSpeed) {         
  analogWrite (speedPin_M2, leftSpeed);
  digitalWrite(directionPin_M1, LOW);
  analogWrite (speedPin_M1, rightSpeed);
  digitalWrite(directionPin_M2, LOW);
}
```

## Partie 2 : Suivi de Ligne avec la Caméra (Line Tracking) 
Pour bien configuré la fonction "line tracking " du Huskylens:https://wiki.dfrobot.com/sen0305/docs/22638

### Description
Cette deuxième partie utilise la caméra intelligente **HuskyLens** configurée avec l'algorithme de suivi de ligne (*Line Tracking*). Au lieu de détecter de simples obstacles, le robot doit maintenant analyser une trajectoire au sol (représentée par une flèche directionnelle sur l'écran de la caméra) et adapter sa direction en temps réel pour rester parfaitement centré sur la ligne.

### Fonctionnement et Logique des États (Machine à États)

Pour fluidifier le déplacement, le programme est structuré autour d'une **machine à états** (`enum State`) qui gère 4 situations distinctes :

1. **L'état `FOLLOW` (Suivi de ligne) :**
   Tant que l'axe de la ligne se trouve dans la zone centrale de l'écran (définie par `DEAD_ZONE`), le robot avance en ligne droite à sa vitesse de croisière.

2. **L'état `ADJUST_LEFT` (Ajustement à gauche) :**
   Si la ligne se décale vers la gauche de l'écran, le robot ralentit sa roue gauche et accélère sa roue droite pour pivoter délicatement vers la gauche et se recentrer.

3. **L'état `ADJUST_RIGHT` (Ajustement à droite) :**
   Si la ligne se décale vers la droite, le robot fait l'inverse : il accélère la roue gauche et ralentit la roue droite pour se recadrer vers la droite.

4. **L'état `LOST` (Sécurité en cas de perte de ligne) :**
   C'est la grande force de ce code. Si le robot rencontre un virage en épingle ou un angle trop aigu (inférieur à 90°) et que la ligne disparaît de l'écran, le robot ne s'arrête pas. Grâce à la variable `lastDirection`, il se souvient de l'endroit où la ligne a été vue pour la dernière fois :
   - Si la ligne a disparu vers la droite, il recule en pivotant vers la droite.
   - Si la ligne a disparu vers la gauche, il recule en pivotant vers la gauche.
   Cette marche arrière en arc de cercle permet à la caméra de retrouver la ligne automatiquement et de relancer le suivi.

### Visuels du Suivi de Ligne
### Code Arduino
```cpp
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
```



## Partie 3 : Suivi d'un Objet avec la Caméra (Object Tracking & Color Recognition)

### Description
Cette troisième et dernière partie exploite les modes de reconnaissance avancés de la caméra **HuskyLens** (*Object Tracking* ou *Color Recognition*). L'objectif est de faire réagir le robot à la présence d'un objet spécifique (une **feuille verte** enregistrée sous l'**ID 1**). Le robot doit suivre l'objet s'il bouge, mais possède également une fonction de sécurité critique : s'arrêter instantanément dès que l'objet disparaît de son champ de vision.

### Fonctionnement et Logique de Sécurité

Le programme analyse en continu les données envoyées par la HuskyLens et applique la logique suivante :

1. **Filtrage par ID (Sécurité anti-reflets) :**
   Pour éviter que le robot ne suive des couleurs "fantômes" ou des reflets résiduels dans la pièce, le code valide uniquement les objets portant l'**ID 1** (`result.ID == 1`). Si une autre couleur verte apparaît, elle est automatiquement ignorée.

2. **Suivi dynamique et centrage :**
   Tant que l'objet ID 1 est détecté (`targetFound = true`), le robot calcule l'écart (`error`) entre le centre de la feuille verte et le centre de l'écran (`CENTER = 160`) :
   - **Bien centré :** Le robot avance droit devant lui à une vitesse stable (`SPEED_STRAIGHT = 110`).
   - **Décalage à gauche :** Le robot ralentit la roue gauche (`SPEED_TURN_MIN`) et accélère la roue droite (`SPEED_TURN_MAX`) pour pivoter vers l'objet.
   - **Décalage à droite :** Le robot effectue la manœuvre inverse pour se recadrer vers la droite.

3. **Arrêt immédiat (Perte de l'objet) :**
   C'est la correction majeure de cette partie. Si l'objet est retiré ou caché, la variable `targetFound` bascule sur `false`. Le programme bascule immédiatement dans le bloc de sécurité `else` et appelle la fonction `motorStop()`. Les moteurs sont coupés instantanément, empêchant le robot de continuer sa course à l'aveugle.

### Visuels de la Détection (Feuille Verde)
### Code Arduino
```cpp
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

  // ─── LOGIQUE DE DÉCISION CORRIGÉE ──────────────────────────
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
