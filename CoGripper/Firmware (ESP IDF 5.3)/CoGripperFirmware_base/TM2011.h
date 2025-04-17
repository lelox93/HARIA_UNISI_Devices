#ifndef TM2011_H
#define TM2011_H

/**
 * @brief Enumerazione che rappresenta gli stati dei pulsanti.
 */
enum ButtonState {
  CW_25DEG,        ///< Pulsante interno: rotazione +25°
  PUSH,            ///< Pulsante interno: pressione (push)
  CCW_25DEG,       ///< Pulsante interno: rotazione -25°
  UNKNOWN,         ///< Nessun pulsante attivo o stato non riconosciuto
  EXT_BUT_1,       ///< Pulsante esterno 1 attivo
  EXT_BUT_2,       ///< Pulsante esterno 2 attivo
  EXT_BUT_1_AND_2  ///< Entrambi i pulsanti esterni attivi contemporaneamente
};

/**
 * @brief Classe per la gestione dei pulsanti del dispositivo TM2011.
 *
 * Questa classe si occupa della configurazione e lettura dei pulsanti interni ed esterni.
 */
class TM2011 {
private:
  const int cw_25deg_pin = 18;   ///< Pin per il pulsante interno +25°
  const int push_pin = 19;       ///< Pin per il pulsante interno "push"
  const int ccw_25deg_pin = 23;  ///< Pin per il pulsante interno -25°
  const int ext_but_1_pin = 5;   ///< Pin per il pulsante esterno 1 (configurato con pull-up)
  const int ext_but_2_pin = 14;  ///< Pin per il pulsante esterno 2 (configurato con pull-up)

public:
  /**
   * @brief Costruttore della classe TM2011.
   *
   * Configura i pin associati ai pulsanti interni ed esterni.
   */
  TM2011();

  /**
   * @brief Legge lo stato dei pulsanti interni.
   *
   * Effettua la lettura digitale dei pin associati ai pulsanti interni.
   *
   * @return ButtonState Stato corrente dei pulsanti interni.
   */
  ButtonState readInternalButtonState();

  /**
   * @brief Legge lo stato dei pulsanti esterni.
   *
   * Effettua la lettura digitale dei pin associati ai pulsanti esterni.
   *
   * @return ButtonState Stato corrente dei pulsanti esterni.
   */
  ButtonState readExternalButtonState();

  /**
   * @brief Ottiene lo stato attuale dei pulsanti interni.
   *
   * Legge lo stato interno e, se abilitato, stampa il risultato sul Serial Monitor per il debug.
   *
   * @param debug_print Se true, abilita la stampa del risultato sul Serial Monitor.
   * @return ButtonState Stato corrente dei pulsanti interni.
   */
  ButtonState getInternalButtonState(bool debug_print = false);

  /**
   * @brief Ottiene lo stato attuale dei pulsanti esterni.
   *
   * Legge lo stato esterno e, se abilitato, stampa il risultato sul Serial Monitor per il debug.
   *
   * @param debug_print Se true, abilita la stampa del risultato sul Serial Monitor.
   * @return ButtonState Stato corrente dei pulsanti esterni.
   */
  ButtonState getExternalButtonState(bool debug_print = false);
};

#endif
