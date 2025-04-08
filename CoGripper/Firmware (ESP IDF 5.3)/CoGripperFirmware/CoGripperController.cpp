#include "CoGripperController.h"

/**
 * @brief Costruttore del CoGripperController.
 * 
 * Inizializza la comunicazione seriale, la connessione Dynamixel e crea le istanze dei due motori.
 *
 * @param txPin Pin TX per la comunicazione seriale.
 * @param rxPin Pin RX per la comunicazione seriale.
 * @param dirPin Pin di direzione per il controllo Dynamixel.
 */
CoGripperController::CoGripperController(uint8_t txPin, uint8_t rxPin, uint8_t dirPin)
  : dynaSerial(1), dxl(dynaSerial, dirPin), motor1(1, dxl), motor2(2, dxl), status(CoGripperStatus::OPEN) {
  // Inizializza la seriale dedicata alla comunicazione con i motori
  dynaSerial.begin(57600, SERIAL_8N1, txPin, rxPin);
  // Inizializza l'interfaccia Dynamixel
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
}

/**
 * @brief Inizializza i motori del gripper.
 *
 * Chiama il metodo initialize() su ciascun motore per configurarli correttamente.
 * Se per un motore con modello "Dynamixel MX-28 2.0" non si riesce a impostare 12V con 3A,
 * il metodo restituisce false.
 */
bool CoGripperController::initialize() {
  // Inizializza il Power Controller
  if (powerController.begin()) {
    Serial.println("PowerController initialized.");
  } else {
    Serial.println("Failed to initialize PowerController.");
    delay(2000);
    Serial.println("Trying to use external power source.");
    // Inizializza i motori
    motor1.initialize();
    motor2.initialize();
    return false;
  }

  bool fallbackOccurred = false;

  // Ottieni il modello del motore
  String motorModel = motor1.getMotorModelNumber();
  Serial.print("Motor 1 model number is: ");
  Serial.println(motorModel);

  // Selezione della tensione basata sul modello del motore
  if (motorModel == "Dynamixel MX-28 2.0") {
    // Prova a impostare 12V con 3A.
    fallbackOccurred = false;  // Resetta la variabile
    if (!powerController.selectVoltage(PD_SRC_12V, CURRENT_3_0_A, &fallbackOccurred) || fallbackOccurred) {
      Serial.println("Impostazione a 12V con 3A non riuscita.");
      return false;
    } else {
      Serial.println("Impostazione a 12V con 3A riuscita.");
    }
  } else {
    // Per altri modelli viene impostato 5V (il parametro current viene ignorato)
    if (powerController.selectVoltage(PD_SRC_5V, CURRENT_3_0_A, &fallbackOccurred)) {
      if (fallbackOccurred) {
        Serial.println("Attenzione: è avvenuto un fallback, ma 5V è stato impostato.");
      } else {
        Serial.println("Impostazione a 5V riuscita senza fallback.");
      }
    } else {
      Serial.println("Impostazione a 5V fallita.");
      return false;
    }
  }

  // Inizializza i motori
  motor1.initialize();
  motor2.initialize();

  return true;
}

/**
 * @brief Calibra i limiti di movimento per entrambi i motori.
 *
 * Esegue la routine di calibrazione (calibrateLimits) per ciascun motore.
 */
void CoGripperController::calibrateLimits() {
  motor1.calibrateLimits();
  motor2.calibrateLimits();
}

/**
 * @brief Ciclo di controllo principale del gripper.
 *
 * Aggiorna le posizioni, i carichi e gli stati dei motori, aggiorna la struttura dati interna,
 * gestisce la logica di torque limit e invia il comando per raggiungere le posizioni obiettivo.
 */
void CoGripperController::controlLoop() {
  // Aggiorna le posizioni dei motori.
  motor1.updatePosition();
  motor2.updatePosition();

  // Aggiorna il carico (load) dei motori.
  motor1.updateLoad();
  motor2.updateLoad();

  // Aggiorna lo stato di ciascun motore in base alla posizione e al carico.
  motor1.updateStatus();
  motor2.updateStatus();

  // Aggiorna le strutture dati interne dei motori.
  motor1.updateData();
  motor2.updateData();

  // Sincronizza i dati interni del controller.
  updateData();

  // Gestisce la logica avanzata per evitare lo stallo.
  handleTorqueLimitEnhanced();

  // Comanda i motori per raggiungere la posizione obiettivo.
  motor1.goToGoalPosition();
  motor2.goToGoalPosition();

  // Per il debug, è possibile stampare lo stato e i dati:
  // printGripperStatus();
  // printGripperData();
}

/**
 * @brief Gestisce gli eventi dei pulsanti per il controllo del gripper.
 *
 * In base al tipo di pulsante premuto, incrementa o decrementa la posizione obiettivo
 * di entrambi i motori oppure esegue un toggle tra gli stati OPEN e CLOSED.
 *
 * @param currentState Stato attuale del pulsante.
 */
void CoGripperController::handleButtonStates(ButtonState currentState) {
  switch (currentState) {
    case CW_25DEG:
      // Incrementa la posizione obiettivo di entrambi i motori se entro il limite massimo.
      if (motor1.getGoalPosition() + 10 < motor1.getEndPositionMax()) {
        motor1.setGoalPosition(motor1.getGoalPosition() + 10);
      }
      if (motor2.getGoalPosition() + 10 < motor2.getEndPositionMax()) {
        motor2.setGoalPosition(motor2.getGoalPosition() + 10);
      }
      break;
    case PUSH:
      // Toggle tra gli stati OPEN e CLOSED.
      if (status == CoGripperStatus::OPEN) {
        status = CoGripperStatus::CLOSED;
        closeAllMotors();
      } else if (status == CoGripperStatus::CLOSED) {
        status = CoGripperStatus::OPEN;
        openAllMotors();
      }
      break;
    case CCW_25DEG:
      // Decrementa la posizione obiettivo di entrambi i motori se entro il limite minimo.
      if (motor1.getGoalPosition() - 10 > motor1.getEndPositionMin()) {
        motor1.setGoalPosition(motor1.getGoalPosition() - 10);
      }
      if (motor2.getGoalPosition() - 10 > motor2.getEndPositionMin()) {
        motor2.setGoalPosition(motor2.getGoalPosition() - 10);
      }
      break;
    default:
      break;
  }
}

/**
 * @brief Gestisce la logica per evitare lo stallo (torque limit) in maniera avanzata.
 *
 * Chiama il metodo handleTorqueLimitEnhanced() per ciascun motore.
 */
void CoGripperController::handleTorqueLimitEnhanced() {
  motor1.handleTorqueLimitEnhanced();
  motor2.handleTorqueLimitEnhanced();
}

/**
 * @brief Stampa lo stato corrente dei motori e del gripper sul Serial Monitor.
 *
 * Visualizza informazioni quali posizione attuale, posizione obiettivo, carico, percentuale di posizione,
 * stato di movimento e stato dei motori, oltre allo stato complessivo del gripper.
 */
void CoGripperController::printGripperStatus() {
  MotorStatus status1 = motor1.getStatus();
  MotorStatus status2 = motor2.getStatus();

  Serial.print("Motor 1 - Pos: ");
  Serial.print(motor1.getCurrentPosition());
  Serial.print(", Goal Pos: ");
  Serial.print(motor1.getGoalPosition());
  Serial.print(", Load: ");
  Serial.print(motor1.getCurrentLoad());
  Serial.print(", Pos%: ");
  Serial.print(motor1.getCurrentPositionPercentage());
  Serial.print(", Move: ");
  Serial.print(motor1.getMoving());
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(status1));

  Serial.print("\t\tMotor 2 - Pos: ");
  Serial.print(motor2.getCurrentPosition());
  Serial.print(", Goal Pos: ");
  Serial.print(motor2.getGoalPosition());
  Serial.print(", Load: ");
  Serial.print(motor2.getCurrentLoad());
  Serial.print(", Pos%: ");
  Serial.print(motor2.getCurrentPositionPercentage());
  Serial.print(", Move: ");
  Serial.print(motor2.getMoving());
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(status2));

  Serial.print("\t\tGRIPPER STATUS: ");
  if (status == CoGripperStatus::OPEN) {
    Serial.println("OPEN");
  }
  if (status == CoGripperStatus::CLOSED) {
    Serial.println("CLOSED");
  }
}

/**
 * @brief Stampa i dati aggiornati dei motori del gripper sul Serial Monitor.
 *
 * Visualizza le informazioni contenute nella struttura dati interna del controller.
 */
void CoGripperController::printGripperData() {
  Serial.print("Motor 1 - Pos: ");
  Serial.print(data.motor1.position);
  Serial.print(", Pos%: ");
  Serial.print(data.motor1.positionPercentage);
  Serial.print(", Load: ");
  Serial.print(data.motor1.load);
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(data.motor1.status));

  Serial.print("\t\tMotor 2 - Pos: ");
  Serial.print(data.motor2.position);
  Serial.print(", Pos%: ");
  Serial.print(data.motor2.positionPercentage);
  Serial.print(", Load: ");
  Serial.print(data.motor2.load);
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(data.motor2.status));

  Serial.print("\t\tGRIPPER STATUS: ");
  if (data.status == CoGripperStatus::OPEN) {
    Serial.println("OPEN");
  }
  if (data.status == CoGripperStatus::CLOSED) {
    Serial.println("CLOSED");
  }
}

/**
 * @brief Imposta il valore PWM massimo per entrambi i motori.
 *
 * @param pwmGoal Il valore PWM da impostare.
 */
void CoGripperController::setGripperMaxPWM(int pwmGoal) {
  motor1.setGoalPWM(pwmGoal);
  motor2.setGoalPWM(pwmGoal);
}

/**
 * @brief Aggiorna la struttura dati interna del controller con i dati attuali dei motori.
 */
void CoGripperController::updateData() {
  data.motor1 = motor1.getData();
  data.motor2 = motor2.getData();
  data.status = status;
}

/**
 * @brief Restituisce la struttura dati contenente le informazioni aggiornate del gripper.
 * 
 * @return CoGripperData Struttura dati aggiornata.
 */
CoGripperData CoGripperController::getData() {
  return data;
}

/**
 * @brief Imposta la posizione obiettivo per il motore 1.
 *
 * @param position La posizione desiderata.
 */
void CoGripperController::setGoalPositionMotor1(int position) {
  motor1.setGoalPosition(position);
}

/**
 * @brief Imposta la posizione obiettivo per il motore 2.
 *
 * @param position La posizione desiderata.
 */
void CoGripperController::setGoalPositionMotor2(int position) {
  motor2.setGoalPosition(position);
}

/**
 * @brief Comanda entrambi i motori ad aprirsi completamente.
 */
void CoGripperController::openAllMotors(void) {
  motor1.setGoalPosition(motor1.getEndPositionMax());
  motor2.setGoalPosition(motor2.getEndPositionMax());
}

/**
 * @brief Comanda entrambi i motori a chiudersi completamente.
 */
void CoGripperController::closeAllMotors(void) {
  motor1.setGoalPosition(motor1.getEndPositionMin());
  motor2.setGoalPosition(motor2.getEndPositionMin());
}

/**
 * @brief Ferma entrambi i motori mantenendo la posizione attuale.
 */
void CoGripperController::stopAllMotors(void) {
  motor1.setGoalPosition(motor1.getCurrentPosition());
  motor2.setGoalPosition(motor2.getCurrentPosition());
}

/**
 * @brief Converte lo stato del motore in una stringa descrittiva.
 *
 * @param status Stato del motore.
 * @return const char* Stringa descrittiva dello stato.
 */
const char* CoGripperController::motorStatusToString(MotorStatus status) const {
  switch (status) {
    case MotorStatus::GRASP:
      return "GRASP";
    case MotorStatus::CLOSING:
      return "CLOSING";
    case MotorStatus::OPEN:
      return "OPEN";
    case MotorStatus::OPENING:
      return "OPENING";
    case MotorStatus::HALF_CLOSED:
      return "HALF_CLOSED";
    default:
      return "UNKNOWN";
  }
}
