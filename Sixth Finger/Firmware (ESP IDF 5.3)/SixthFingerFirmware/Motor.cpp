#include "WString.h"
#include "Motor.h"

// Costruttore
Motor::Motor(uint8_t id, Dynamixel2Arduino &dxl)
  : id(id),
    dxl(dxl),
    goalPosition(2300),
    currentPosition(0),
    currentPositionPercentage(0),
    load(0),
    endPositionMin(0),
    endPositionMax(0),
    stallAvoidanceCounter(0),
    stallDetectionCounter(0),
    secondaryGoal(false),
    stallAvoided(false),
    status(MotorStatus::OPEN) {}

// Inizializza le impostazioni del motore
void Motor::initialize() {
  const uint16_t MAX_CURRENT = 880;
  const uint32_t MOVING_THRESHOLD = 5;

  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_EXTENDED_POSITION);
  dxl.write(id, 36, (uint8_t *)&MAX_CURRENT, sizeof(MAX_CURRENT));
  dxl.write(id, 24, (uint8_t *)&MOVING_THRESHOLD, sizeof(MOVING_THRESHOLD));
  delay(500);
  dxl.torqueOn(id);

  // Imposta la velocità per il controllo event-driven
  configureVelocity(HIGH);

  // Sincronizza la posizione corrente con il goal
  currentPosition = dxl.getPresentPosition(id);
  goalPosition = currentPosition;
  dxl.setGoalPosition(id, goalPosition);
}

// Configura velocità e accelerazione del motore
void Motor::configureVelocity(int vel) {
  Serial.print("Motor ");
  Serial.print(id);
  Serial.print(": ");

  uint32_t profile_vel = 0;
  uint32_t profile_acc = 0;

  if (vel == HIGH) {
    profile_vel = 300;
    profile_acc = 250;
  } else if (vel == LOW) {
    profile_vel = 30;
    profile_acc = 25;
  } else if (vel >= 30 && vel <= 300) {
    profile_vel = vel;
    profile_acc = static_cast<uint32_t>(vel * (25.0f / 30.0f));
  }

  // Aggiorna i parametri di velocità e accelerazione sul motore
  while (!dxl.write(id, 112, (uint8_t *)&profile_vel, sizeof(profile_vel))) {
    delay(50);
  }
  while (!dxl.write(id, 108, (uint8_t *)&profile_acc, sizeof(profile_acc))) {
    delay(50);
  }
  Serial.println("velocity change success");
}

// Calibra i limiti di movimento del motore
void Motor::calibrateLimits() {
  currentPosition = dxl.getPresentPosition(id);
  goalPosition = currentPosition + 100;
  dxl.setGoalPosition(id, goalPosition);

  const int THRESHOLD_LOAD = 450;
  Serial.println("SEARCHING LOAD");
  do {
    Serial.print(".");
    goalPosition -= 15;
    dxl.setGoalPosition(id, goalPosition);
    currentPosition = dxl.getPresentPosition(id);
    updateLoad();
  } while (load <= THRESHOLD_LOAD);
  
  Serial.println();
  Serial.println("LOAD FOUND!");
  delay(500);

  goalPosition = currentPosition - 20;
  endPositionMin = goalPosition;
  endPositionMax = goalPosition + 2300;
  
  Serial.print("COMPUTED END POSITION MIN: ");
  Serial.println(endPositionMin);
  delay(200);
  Serial.print("COMPUTED END POSITION MAX: ");
  Serial.println(endPositionMax);
  delay(200);
  
  goalPosition = endPositionMax;
  Serial.println("GOING TO MAX POSITION");
  setGoalPosition(goalPosition);
  goToGoalPosition();
  Serial.println();
  Serial.println("DONE!");
  delay(1000);
}

// Aggiorna la posizione attuale e la percentuale relativa ai limiti calibrati
void Motor::updatePosition() {
  currentPosition = dxl.getPresentPosition(id);
  currentPositionPercentage = map(currentPosition, endPositionMin, endPositionMax, 0, 100);
}

// Aggiorna il valore di carico del motore
void Motor::updateLoad() {
  uint16_t motor_current;
  dxl.read(id, 126, 2, (uint8_t *)&motor_current, sizeof(motor_current));

  if (motor_current >= 1023) {
    motor_current = 65535 - motor_current;
  }
  load = motor_current;
}

// Aggiorna la struttura dati interna del motore
void Motor::updateData() {
  data.load = load;
  data.position = currentPosition;
  data.positionPercentage = currentPositionPercentage;
  data.status = status;
}

// Ritorna la struttura dati del motore
MotorData Motor::getData() {
  return data;
}

// Gestione base del limite di coppia (torque)
void Motor::handleTorqueLimit() {
  const int tolerance = 6;
  
  if (load > 400 && (currentPosition > goalPosition + tolerance)) {
    Serial.println("dyna_1_stall_avoidance_counter");
    stallDetectionCounter++;
  }

  if (stallDetectionCounter > 0) {
    stallDetectionCounter = 0;
    setGoalPosition(currentPosition + 2);
    delay(200);
  }

  if (stallDetectionCounter == 0) {
    setGoalPosition(goalPosition);
  }
}

// Gestione avanzata del limite di coppia
void Motor::handleTorqueLimitEnhanced() {
  const int tolerance = 6;
  
  if (load > 400 && (currentPosition > goalPosition + tolerance)) {
    stallDetectionCounter++;
  }

  if (stallDetectionCounter > 10) {
    stallDetectionCounter = 0;
    setGoalPosition(currentPosition + 2);
    secondaryGoal = true;
  }

  if (secondaryGoal) {
    if (load < 340) {
      setGoalPosition(goalPosition - 1);
    }
    if (load > 360) {
      setGoalPosition(currentPosition + 1);
    }
    if (load > 340 && load < 360) {
      stallAvoidanceCounter++;
      if (stallAvoidanceCounter > 20) {
        stallAvoidanceCounter = 0;
        stallAvoided = true;
      }
    }
    if (stallAvoided || status == MotorStatus::OPENING || status == MotorStatus::OPEN) {
      stallAvoided = false;
      secondaryGoal = false;
    }
  }
}

// Ritorna il carico attuale del motore
int Motor::getCurrentLoad() {
  return load;
}

// Ritorna la posizione attuale del motore
int Motor::getCurrentPosition() const {
  return currentPosition;
}

// Imposta la posizione obiettivo del motore
void Motor::setGoalPosition(int position) {
  goalPosition = position;
}

// Comanda il motore a raggiungere la posizione obiettivo
void Motor::goToGoalPosition() {
  dxl.setGoalPosition(id, goalPosition);
}

// Ritorna la percentuale della posizione attuale rispetto ai limiti
int Motor::getCurrentPositionPercentage() const {
  return currentPositionPercentage;
}

// Imposta il PWM obiettivo per il motore
void Motor::setGoalPWM(int pwmGoal) {
  dxl.setGoalPWM(id, pwmGoal);
}

// Ritorna true se il motore è in movimento
bool Motor::getMoving() const {
  uint8_t moving_val;
  dxl.read(id, 122, 1, (uint8_t *)&moving_val, sizeof(moving_val));
  return (moving_val == 1);
}

// Aggiorna lo stato del motore basandosi su movimento e carico
void Motor::updateStatus() {
  if (getMoving()) {
    if (currentPosition < goalPosition) {
      status = MotorStatus::OPENING;
    } else if (!secondaryGoal) {
      status = MotorStatus::CLOSING;
    }
  } else {
    if (currentPositionPercentage >= 95) {  // Condizione per motore aperto
      status = MotorStatus::OPEN;
    } else {
      if (load < 200 && !secondaryGoal) {  // Condizione per mezzo chiuso
        status = MotorStatus::HALF_CLOSED;
      } else {
        status = MotorStatus::GRASP;
      }
    }
  }
}

// Ritorna lo stato attuale del motore
MotorStatus Motor::getStatus() {
  return status;
}

// Ritorna il modello del motore sotto forma di stringa
String Motor::getMotorModelNumber() {
  int modelNum = dxl.getModelNumber(id);
  String modelStr;
  
  if (modelNum == 30) {
    modelStr = "Dynamixel MX-28 2.0";
  } else if (modelNum == 1200) {
    modelStr = "Dynamixel XL330-M288-T 2.0";
  } else {
    modelStr = "Unknown";
  }
  
  return modelStr;
}
