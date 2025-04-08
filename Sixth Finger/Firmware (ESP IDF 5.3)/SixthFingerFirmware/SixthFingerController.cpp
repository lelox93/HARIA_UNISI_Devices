#include "SixthFingerController.h"

SixthFingerController::SixthFingerController(uint8_t txPin, uint8_t rxPin, uint8_t dirPin)
  : dynaSerial(1),
    dxl(dynaSerial, dirPin),
    motor1(1, dxl),
    status(SixthFingerStatus::OPEN) {
  dynaSerial.begin(57600, SERIAL_8N1, txPin, rxPin);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(2.0);
}

bool SixthFingerController::initialize() {
  // Inizializza il Power Controller
  if (powerController.begin()) {
    Serial.println("PowerController initialized.");
  } else {
    Serial.println("Failed to initialize PowerController.");
    Serial.println("Trying to use ext pwr.");
    motor1.initialize();
    powerController.startTPS2121Task();
    delay(2000);
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
    if (!powerController.selectVoltage(PD_SRC_12V, CURRENT_1_5_A, &fallbackOccurred) || fallbackOccurred) {
      Serial.println("Impostazione a 12V con 1.5A non riuscita.");
      return false;
    } else {
      Serial.println("Impostazione a 12V con 1.5A riuscita.");
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

  // Inizializza NVS in modalità lettura
  preferences.begin("myApp", true);
  global_var_vel = preferences.getInt("global_var_vel", 30);
  Serial.print("Valore letto da NVS: ");
  Serial.println(global_var_vel);
  preferences.end();

  motor1.initialize();
  powerController.startTPS2121Task();
}

void SixthFingerController::calibrateLimits() {
  // Aggiorna i nomi: limitsCalibration -> calibrateLimits e changeMotorVelocity -> configureVelocity
  motor1.calibrateLimits();
  motor1.configureVelocity(global_var_vel);
}

void SixthFingerController::controlLoop() {
  motor1.updatePosition();
  motor1.updateLoad();
  motor1.updateStatus();
  motor1.updateData();
  updateData();
  handleTorqueLimitEnhanced();
  motor1.goToGoalPosition();
  // printGripperStatus();
  // printGripperData();
}

void SixthFingerController::handleButtonStates(ButtonState currentState) {
  unsigned long currentTime = millis();

  switch (currentState) {
    case ButtonState::CW_25DEG:
      if (motor1.getGoalPosition() + 10 < motor1.getEndPositionMax()) {
        motor1.setGoalPosition(motor1.getGoalPosition() + 10);
      }
      break;
    case ButtonState::PUSH:
      global_var_vel = (global_var_vel + 20) % 111;
      if (global_var_vel == 0 || global_var_vel == 1) {
        global_var_vel = 30;
      }
      motor1.configureVelocity(global_var_vel);

      // Salva il nuovo valore su NVS
      preferences.begin("myApp", false);
      preferences.putInt("global_var_vel", global_var_vel);
      preferences.end();
      delay(debounceDelay);
      /*
      // Meccanismo di toggle per apertura/chiusura (attualmente disabilitato)
      if (currentTime - lastPushTime >= debounceDelay) {
        if (status == SixthFingerStatus::OPEN) {
          status = SixthFingerStatus::CLOSED;
          closeAllMotors();
        } else if (status == SixthFingerStatus::CLOSED) {
          status = SixthFingerStatus::OPEN;
          openAllMotors();
        }
        lastPushTime = currentTime;
      }
      */
      break;
    case ButtonState::CCW_25DEG:
      if (motor1.getGoalPosition() - 10 > motor1.getEndPositionMin()) {
        motor1.setGoalPosition(motor1.getGoalPosition() - 10);
      }
      break;
    default:
      break;
  }
}

// Rinominato da handleTorqueLimitStatus a handleTorqueLimitEnhanced
void SixthFingerController::handleTorqueLimitEnhanced() {
  motor1.handleTorqueLimitEnhanced();
}

void SixthFingerController::printGripperStatus() {
  MotorStatus mStatus = motor1.getStatus();

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
  Serial.print(motorStatusToString(mStatus));

  Serial.print("\tFINGER STATUS: ");
  Serial.println((status == SixthFingerStatus::OPEN) ? "OPEN" : "CLOSED");
}

void SixthFingerController::printGripperData() {
  Serial.print("Motor 1 - Pos: ");
  Serial.print(data.motor1.position);
  Serial.print(", Pos%: ");
  Serial.print(data.motor1.positionPercentage);
  Serial.print(", Load: ");
  Serial.print(data.motor1.load);
  Serial.print(", Status: ");
  Serial.print(motorStatusToString(data.motor1.status));

  Serial.print("\tFINGER STATUS: ");
  Serial.println((data.status == SixthFingerStatus::OPEN) ? "OPEN" : "CLOSED");
}

void SixthFingerController::setSixthFingerMaxPWM(int pwmGoal) {
  motor1.setGoalPWM(pwmGoal);
}

void SixthFingerController::updateData() {
  data.motor1 = motor1.getData();
  data.status = status;
}

SixthFingerData SixthFingerController::getData() {
  return data;
}

void SixthFingerController::setGoalPositionMotor1(int position) {
  motor1.setGoalPosition(position);
}

void SixthFingerController::openAllMotors() {
  motor1.setGoalPosition(motor1.getEndPositionMax());
}

void SixthFingerController::closeAllMotors() {
  motor1.setGoalPosition(motor1.getEndPositionMin());
}

void SixthFingerController::stopAllMotors() {
  motor1.setGoalPosition(motor1.getCurrentPosition());
}

const char* SixthFingerController::motorStatusToString(MotorStatus status) const {
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

void SixthFingerController::handleExtButtonStates(ButtonState currentState) {
  switch (currentState) {
    case EXT_BUT_1:
      closeAllMotors();
      break;
    case EXT_BUT_2:
      openAllMotors();
      break;
    case EXT_BUT_1_AND_2:
      // Nessuna azione
      break;
    case UNKNOWN:
      // Nessuna azione
      break;
  }
}
