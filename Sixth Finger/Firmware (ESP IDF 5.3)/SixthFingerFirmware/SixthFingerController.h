#ifndef SIXTHFINGER_CONTROLLER_H
#define SIXTHFINGER_CONTROLLER_H

#include <Dynamixel2Arduino.h>
#include <HardwareSerial.h>
#include "TM2011.h"
#include "Motor.h"
#include "PowerController.h"
#include <Preferences.h>  // Libreria per la gestione della memoria non volatile (NVS)

enum class SixthFingerStatus {
  CLOSED,  ///< Stato: chiuso
  OPEN     ///< Stato: aperto
};

/// Struttura contenente i dati aggiornati del controller e del motore.
struct SixthFingerData {
  MotorData motor1;                    ///< Dati specifici del motore
  SixthFingerStatus status = SixthFingerStatus::OPEN;  ///< Stato corrente del controller
};

class SixthFingerController {
public:
  /**
   * @brief Costruttore del SixthFingerController.
   * @param txPin Pin TX per la comunicazione seriale.
   * @param rxPin Pin RX per la comunicazione seriale.
   * @param dirPin Pin di direzione per il controllo del motore.
   */
  SixthFingerController(uint8_t txPin, uint8_t rxPin, uint8_t dirPin);

  /**
   * @brief Inizializza il controller.
   *
   * Configura il Power Controller, seleziona la tensione in base al modello del motore,
   * inizializza la memoria non volatile (NVS) e sincronizza i parametri del motore.
   */
  bool initialize();

  /**
   * @brief Calibra i limiti di movimento del motore.
   *
   * Determina le posizioni minime e massime raggiungibili dal motore e configura la velocità.
   */
  void calibrateLimits();

  /**
   * @brief Esegue il ciclo di controllo principale.
   *
   * Aggiorna le posizioni, i carichi e lo stato del motore, gestisce la logica di
   * torque limit e invia il comando per raggiungere la posizione obiettivo.
   */
  void controlLoop();

  /**
   * @brief Gestisce gli stati dei pulsanti interni per il controllo del motore.
   * @param currentState Stato corrente del pulsante.
   */
  void handleButtonStates(ButtonState currentState);

  /**
   * @brief Gestisce gli stati dei pulsanti esterni per il controllo diretto del motore.
   * @param currentState Stato corrente del pulsante.
   */
  void handleExtButtonStates(ButtonState currentState);

  /**
   * @brief Gestisce la logica avanzata per evitare situazioni di stallo (torque limit).
   */
  void handleTorqueLimitEnhanced();

  /**
   * @brief Aggiorna la struttura dati interna del controller.
   */
  void updateData();

  /**
   * @brief Restituisce una struttura contenente i dati aggiornati del controller.
   * @return SixthFingerData con i dati del motore e lo stato del controller.
   */
  SixthFingerData getData();

  /**
   * @brief Imposta il valore PWM massimo per il motore.
   * @param pwmGoal Valore PWM da impostare.
   */
  void setSixthFingerMaxPWM(int pwmGoal);

  /**
   * @brief Imposta la posizione obiettivo del motore.
   * @param position Nuova posizione obiettivo per il motore.
   */
  void setGoalPositionMotor1(int position);

  /**
   * @brief Comanda il motore a spostarsi verso la posizione massima (apertura completa).
   */
  void openAllMotors();

  /**
   * @brief Comanda il motore a spostarsi verso la posizione minima (chiusura completa).
   */
  void closeAllMotors();

  /**
   * @brief Ferma il motore mantenendo la posizione attuale.
   */
  void stopAllMotors();

  /**
   * @brief Converte lo stato del motore in una stringa descrittiva.
   * @param status Stato del motore.
   * @return Stringa contenente la descrizione dello stato.
   */
  const char* motorStatusToString(MotorStatus status) const;

  /**
   * @brief Stampa sul Serial Monitor lo stato corrente del motore e del controller.
   */
  void printGripperStatus();

  /**
   * @brief Stampa sul Serial Monitor i dati aggiornati del motore.
   */
  void printGripperData();

  /**
   * @brief Ritorna il limite inferiore del movimento del motore.
   * @return Posizione minima raggiungibile dal motore.
   */
  int getMotor1EndPositionMin() const { return motor1.getEndPositionMin(); }

  /**
   * @brief Ritorna il limite superiore del movimento del motore.
   * @return Posizione massima raggiungibile dal motore.
   */
  int getMotor1EndPositionMax() const { return motor1.getEndPositionMax(); }

  /// Istanza del motore gestito dal controller.
  Motor motor1;

private:
  PowerController powerController;  ///< Gestisce la selezione della tensione in ingresso
  HardwareSerial dynaSerial;        ///< Serial utilizzata per comunicare con i motori
  Dynamixel2Arduino dxl;            ///< Interfaccia per il controllo dei motori Dynamixel
  SixthFingerStatus status;         ///< Stato corrente del controller (OPEN/CLOSED)
  SixthFingerData data;             ///< Dati aggiornati del controller e del motore
  Preferences preferences;          ///< Gestione della memoria non volatile (NVS)

  unsigned long lastPushTime = 0;   ///< Timestamp dell'ultimo evento di pressione (per debounce)
  const unsigned long debounceDelay = 500;  ///< Ritardo per il debounce (in millisecondi)
  int global_var_vel;               ///< Velocità globale del motore, salvata in NVS
};

#endif  // SIXTHFINGER_CONTROLLER_H
