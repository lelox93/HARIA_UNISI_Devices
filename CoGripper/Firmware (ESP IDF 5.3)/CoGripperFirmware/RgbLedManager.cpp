#include "RgbLedManager.h"

// Costruttore: inizializza la striscia LED e i parametri interni.
RgbLedManager::RgbLedManager(uint16_t numPixels, uint8_t pin, uint32_t type)
  : _strip(numPixels, pin, type), _mode(LedEffectMode::OFF),
    _currentR(0), _currentG(0), _currentB(0),
    _fadeStartR(0), _fadeStartG(0), _fadeStartB(0),
    _fadeTargetR(0), _fadeTargetG(0), _fadeTargetB(0),
    _fadeDuration(0), _fadeStartTime(0),
    _blinkR(0), _blinkG(0), _blinkB(0), _blinkInterval(0),
    _blinkOn(false), _lastBlinkTime(0),
    _sineR(0), _sineG(0), _sineB(0), _sineFrequency(0),
    _sineStartTime(0), _ledTaskHandle(NULL)
{}

// Inizializza la striscia LED e avvia il task per l'aggiornamento degli effetti.
void RgbLedManager::begin() {
  _strip.begin();
  _strip.show();
  // Avvia il task FreeRTOS che gestisce l'aggiornamento del LED.
  xTaskCreate(ledTask, "LedTask", 1024, this, 1, &_ledTaskHandle);
}

// Imposta il LED in modalità statica con il colore specificato.
void RgbLedManager::setStaticColor(uint8_t r, uint8_t g, uint8_t b) {
  _mode = LedEffectMode::STATIC;
  _currentR = r;
  _currentG = g;
  _currentB = b;
  _strip.setPixelColor(0, _strip.Color(r, g, b));
  _strip.show();
}

// Avvia una transizione lineare (fade) verso il colore target in un determinato intervallo.
void RgbLedManager::fadeToColor(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms) {
  _mode = LedEffectMode::FADE;
  // Imposta il colore di partenza come quello corrente.
  _fadeStartR = _currentR;
  _fadeStartG = _currentG;
  _fadeStartB = _currentB;
  // Imposta il colore target.
  _fadeTargetR = r;
  _fadeTargetG = g;
  _fadeTargetB = b;
  _fadeDuration = duration_ms;
  _fadeStartTime = millis();
}

// Imposta un effetto di lampeggio con il colore e l'intervallo specificato.
void RgbLedManager::setBlink(uint8_t r, uint8_t g, uint8_t b, uint32_t interval_ms) {
  _mode = LedEffectMode::BLINK;
  _blinkR = r;
  _blinkG = g;
  _blinkB = b;
  _blinkInterval = interval_ms;
  _blinkOn = false; // Inizia con il LED spento.
  _lastBlinkTime = millis();
}

// Avvia un effetto di fading sinusoidale con il colore base e la frequenza specificata.
void RgbLedManager::setSinusoidalEffect(uint8_t r, uint8_t g, uint8_t b, float frequency) {
  _mode = LedEffectMode::SINUSOIDAL;
  _sineR = r;
  _sineG = g;
  _sineB = b;
  _sineFrequency = frequency;
  _sineStartTime = millis();
}

// Spegne il LED.
void RgbLedManager::turnOff() {
  _mode = LedEffectMode::OFF;
  _currentR = 0; _currentG = 0; _currentB = 0;
  _strip.setPixelColor(0, _strip.Color(0,0,0));
  _strip.show();
}

// Loop di aggiornamento eseguito dal task FreeRTOS.
// In base alla modalità corrente, calcola e applica l'effetto desiderato.
void RgbLedManager::updateLoop() {
  while (true) {
    uint32_t now = millis();
    switch (_mode) {
      case LedEffectMode::OFF:
        _strip.setPixelColor(0, _strip.Color(0,0,0));
        _strip.show();
        break;
      case LedEffectMode::STATIC:
        // Modalità statica: non occorre aggiornare continuamente.
        break;
      case LedEffectMode::FADE: {
        uint32_t elapsed = now - _fadeStartTime;
        if (elapsed >= _fadeDuration) {
          // Fine della transizione: imposta il colore target e passa in STATIC.
          _currentR = _fadeTargetR;
          _currentG = _fadeTargetG;
          _currentB = _fadeTargetB;
          _mode = LedEffectMode::STATIC;
        } else {
          float progress = (float) elapsed / _fadeDuration;
          _currentR = _fadeStartR + (uint8_t)((_fadeTargetR - _fadeStartR) * progress);
          _currentG = _fadeStartG + (uint8_t)((_fadeTargetG - _fadeStartG) * progress);
          _currentB = _fadeStartB + (uint8_t)((_fadeTargetB - _fadeStartB) * progress);
        }
        _strip.setPixelColor(0, _strip.Color(_currentR, _currentG, _currentB));
        _strip.show();
        break;
      }
      case LedEffectMode::BLINK: {
        if (now - _lastBlinkTime >= _blinkInterval) {
          _blinkOn = !_blinkOn;
          _lastBlinkTime = now;
        }
        if (_blinkOn)
          _strip.setPixelColor(0, _strip.Color(_blinkR, _blinkG, _blinkB));
        else
          _strip.setPixelColor(0, _strip.Color(0,0,0));
        _strip.show();
        break;
      }
      case LedEffectMode::SINUSOIDAL: {
        float t = (now - _sineStartTime) / 1000.0; // Tempo in secondi
        float brightnessFactor = 0.5 * (1 + sin(2 * PI * _sineFrequency * t));
        uint8_t r = _sineR * brightnessFactor;
        uint8_t g = _sineG * brightnessFactor;
        uint8_t b = _sineB * brightnessFactor;
        _strip.setPixelColor(0, _strip.Color(r, g, b));
        _strip.show();
        break;
      }
      default:
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(20));  // Aggiornamento ogni 20ms
  }
}

// Funzione statica wrapper per il task LED.
void RgbLedManager::ledTask(void* param) {
  RgbLedManager* self = static_cast<RgbLedManager*>(param);
  self->updateLoop();
}
