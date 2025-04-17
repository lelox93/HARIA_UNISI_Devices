#ifndef MOTOR_H
#define MOTOR_H

#include <Dynamixel2Arduino.h>

// Questo namespace è necessario per utilizzare i nomi degli elementi della control table
using namespace ControlTableItem;

// Enum per rappresentare lo stato attuale del motore.
enum class MotorStatus {
  GRASP,        // Il motore ha afferrato: presa completata
  CLOSING,      // Il motore si sta chiudendo
  OPEN,         // Il motore è completamente aperto
  HALF_CLOSED,  // Il motore è chiuso parzialmente (senza presa completa)
  OPENING       // Il motore si sta aprendo
};

// Struttura per memorizzare i dati rilevanti del motore.
struct MotorData {
  int load = 0;                  // Valore attuale del carico (coppia)
  int position = 0;              // Posizione attuale del motore
  int positionPercentage = 0;    // Posizione espressa in percentuale rispetto ai limiti calcolati
  MotorStatus status = MotorStatus::OPEN;  // Stato corrente del motore
};

class Motor {
public:
  /**
   * Costruttore del motore.
   * @param id Identificativo univoco del motore.
   * @param dxl Riferimento all'istanza Dynamixel2Arduino per la comunicazione con il motore.
   */
  Motor(uint8_t id, Dynamixel2Arduino& dxl);

  /**
   * Inizializza il motore configurando la modalità operativa e impostando i parametri iniziali.
   */
  void initialize();

  /**
   * Calibra i limiti di movimento del motore, determinando le posizioni minime e massime.
   */
  void calibrateLimits();

  /**
   * Aggiorna la posizione corrente del motore e calcola la posizione in percentuale rispetto ai limiti.
   */
  void updatePosition();

  /**
   * Legge e aggiorna il valore del carico (load) del motore.
   */
  void updateLoad();

  /**
   * Gestisce le condizioni di sovraccarico (limite di coppia) e regola la posizione obiettivo se necessario.
   */
  void handleTorqueLimit();

  /**
   * Gestione avanzata del limite di coppia, che include una strategia secondaria per evitare lo stallo.
   */
  void handleTorqueLimitEnhanced();

  /**
   * Configura la velocità e l'accelerazione del motore.
   * @param vel Valore della velocità da impostare. Valori predefiniti: HIGH, LOW oppure un valore compreso tra 30 e 300.
   */
  void configureVelocity(int vel);

  /**
   * Ritorna il carico attuale del motore.
   * @return Valore del carico.
   */
  int getCurrentLoad();

  /**
   * Ritorna la posizione attuale del motore.
   * @return Posizione corrente.
   */
  int getCurrentPosition() const;

  /**
   * Ritorna la posizione obiettivo corrente del motore.
   * @return Posizione obiettivo.
   */
  int getGoalPosition() const { return goalPosition; }

  /**
   * Ritorna il limite inferiore della posizione calcolato durante la calibrazione.
   * @return Posizione minima.
   */
  int getEndPositionMin() const { return endPositionMin; }

  /**
   * Ritorna il limite superiore della posizione calcolato durante la calibrazione.
   * @return Posizione massima.
   */
  int getEndPositionMax() const { return endPositionMax; }

  /**
   * Imposta la posizione obiettivo del motore.
   * @param position Nuova posizione obiettivo.
   */
  void setGoalPosition(int position);

  /**
   * Comanda il motore a raggiungere la posizione obiettivo.
   */
  void goToGoalPosition();

  /**
   * Ritorna la posizione attuale espressa in percentuale rispetto ai limiti calcolati.
   * @return Percentuale della posizione.
   */
  int getCurrentPositionPercentage() const;

  /**
   * Restituisce il modello del motore sotto forma di stringa.
   * @return Stringa contenente il modello del motore.
   */
  String getMotorModelNumber();

  /**
   * Ritorna l'identificativo univoco del motore.
   * @return ID del motore.
   */
  uint8_t getID() const { return id; }

  /**
   * Imposta il valore PWM obiettivo per il motore.
   * @param pwmGoal Valore PWM da impostare.
   */
  void setGoalPWM(int pwmGoal);

  /**
   * Verifica se il motore è attualmente in movimento.
   * @return true se il motore è in movimento, altrimenti false.
   */
  bool getMoving() const;

  /**
   * Ritorna lo stato attuale del motore.
   * @return Stato corrente.
   */
  MotorStatus getStatus();

  /**
   * Aggiorna lo stato del motore basandosi su posizione, movimento e carico.
   */
  void updateStatus();

  /**
   * Ritorna una struttura dati contenente tutte le informazioni rilevanti del motore.
   * @return Struttura MotorData con i dati aggiornati.
   */
  MotorData getData();

  /**
   * Aggiorna la struttura dati interna del motore.
   */
  void updateData();

private:
  uint8_t id;                   // Identificativo del motore
  Dynamixel2Arduino& dxl;       // Riferimento per la comunicazione tramite Dynamixel2Arduino
  int goalPosition;             // Posizione obiettivo corrente
  int currentPosition;          // Posizione attuale del motore
  int currentPositionPercentage;// Posizione attuale espressa in percentuale rispetto ai limiti
  int load;                     // Valore attuale del carico (coppia)
  int endPositionMin;           // Limite inferiore della posizione (calibrato)
  int endPositionMax;           // Limite superiore della posizione (calibrato)
  int stallAvoidanceCounter;    // Contatore per il meccanismo di evitamento dello stallo
  int stallDetectionCounter;    // Contatore per la rilevazione dello stallo
  bool secondaryGoal;           // Flag per attivare una strategia secondaria di movimento in caso di stallo
  bool stallAvoided;            // Flag per indicare se lo stallo è stato evitato
  MotorStatus status;           // Stato corrente del motore
  MotorData data;               // Struttura dati contenente le informazioni rilevanti del motore
};

#endif  // MOTOR_H
