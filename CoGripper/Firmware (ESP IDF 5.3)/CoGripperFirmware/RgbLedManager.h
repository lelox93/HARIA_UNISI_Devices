#ifndef RGB_LEDMANAGER_H
#define RGB_LEDMANAGER_H

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Enumerazione dei possibili effetti LED.
 */
enum class LedEffectMode {
  OFF,         ///< LED spento
  STATIC,      ///< Colore statico
  FADE,        ///< Transizione lineare da un colore a un altro
  BLINK,       ///< Lampeggio con intervallo fisso
  SINUSOIDAL   ///< Modulazione sinusoidale della luminosità
};

/**
 * @brief Classe per la gestione dinamica di un LED RGB tramite FreeRTOS.
 *
 * Questa classe incapsula la logica per impostare il colore, effettuare transizioni (fade),
 * lampeggiare o applicare un effetto di fading sinusoidale, aggiornando periodicamente il LED.
 */
class RgbLedManager {
public:
  /**
   * @brief Costruttore della classe RgbLedManager.
   * @param numPixels Numero di LED presenti nella striscia.
   * @param pin Pin a cui è collegata la striscia LED.
   * @param type Tipo di LED (default: NEO_GRB + NEO_KHZ800).
   */
  RgbLedManager(uint16_t numPixels, uint8_t pin, uint32_t type = NEO_GRB + NEO_KHZ800);

  /**
   * @brief Inizializza la striscia LED e avvia il task per l'aggiornamento degli effetti.
   */
  void begin();

  /**
   * @brief Imposta il LED in modalità statica con il colore specificato.
   * @param r Valore rosso (0-255).
   * @param g Valore verde (0-255).
   * @param b Valore blu (0-255).
   */
  void setStaticColor(uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Avvia una transizione lineare (fade) verso il colore target.
   * @param r Valore rosso target (0-255).
   * @param g Valore verde target (0-255).
   * @param b Valore blu target (0-255).
   * @param duration_ms Durata della transizione in millisecondi.
   */
  void fadeToColor(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms);

  /**
   * @brief Imposta un effetto di lampeggio.
   * @param r Valore rosso (0-255).
   * @param g Valore verde (0-255).
   * @param b Valore blu (0-255).
   * @param interval_ms Intervallo del lampeggio in millisecondi.
   */
  void setBlink(uint8_t r, uint8_t g, uint8_t b, uint32_t interval_ms);

  /**
   * @brief Avvia un effetto di fading sinusoidale.
   * @param r Valore rosso base (0-255).
   * @param g Valore verde base (0-255).
   * @param b Valore blu base (0-255).
   * @param frequency Frequenza dell'effetto in Hz.
   */
  void setSinusoidalEffect(uint8_t r, uint8_t g, uint8_t b, float frequency);

  /**
   * @brief Spegne il LED.
   */
  void turnOff();

private:
  Adafruit_NeoPixel _strip;       ///< Istanza della striscia LED
  LedEffectMode _mode;            ///< Modalità effetto corrente
  
  // Colore corrente (usato in STATIC e come base per gli effetti)
  uint8_t _currentR, _currentG, _currentB;
  
  // Parametri per il fade
  uint8_t _fadeStartR, _fadeStartG, _fadeStartB;  ///< Colore di partenza della transizione
  uint8_t _fadeTargetR, _fadeTargetG, _fadeTargetB; ///< Colore target della transizione
  uint32_t _fadeDuration;     ///< Durata del fade in millisecondi
  uint32_t _fadeStartTime;    ///< Tempo di inizio della transizione (millis)
  
  // Parametri per il lampeggio
  uint8_t _blinkR, _blinkG, _blinkB;  ///< Colore usato nel lampeggio
  uint32_t _blinkInterval;    ///< Intervallo di lampeggio in millisecondi
  bool _blinkOn;              ///< Stato corrente del lampeggio (acceso/spento)
  uint32_t _lastBlinkTime;    ///< Ultimo aggiornamento del lampeggio (millis)
  
  // Parametri per l'effetto sinusoidale
  uint8_t _sineR, _sineG, _sineB;  ///< Colore base per l'effetto sinusoidale
  float _sineFrequency;           ///< Frequenza dell'effetto in Hz
  uint32_t _sineStartTime;        ///< Tempo di inizio effetto (millis)
  
  TaskHandle_t _ledTaskHandle;    ///< Handle del task per l'aggiornamento LED
  
  /**
   * @brief Loop di aggiornamento eseguito dal task FreeRTOS.
   *
   * Aggiorna periodicamente il LED in base alla modalità selezionata.
   */
  void updateLoop();

  /**
   * @brief Funzione statica wrapper per il task LED.
   * @param param Puntatore all'istanza di RgbLedManager.
   */
  static void ledTask(void* param);
};

#endif // RGB_LEDMANAGER_H
