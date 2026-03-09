// Flash Light Module for NodeMCU V3
// PWM-based lamp control with multiple modes
// Supports HW-095 ESC (2-pin control: IN1, IN2)

#include "flash_light.h"

// Walk up brightness values (0-255)
static const uint8_t walkValues[] = {10, 40, 60, 100, 150, 200, 255};
static const uint8_t walkSteps = sizeof(walkValues) / sizeof(walkValues[0]);

FlashLight::FlashLight(uint8_t pinIn1, uint8_t pinIn2)
    : _pinIn1(pinIn1)
    , _pinIn2(pinIn2)
    , _mode(FLASH_MODE_OFF)
    , _brightness(255)
    , _currentPwm(0)
    , _lastUpdate(0)
    , _strobeOnMs(200)
    , _strobeOffMs(200)
    , _walkIntervalMs(1000)
    , _strobeState(false)
    , _walkStep(0)
{
}

void FlashLight::begin() {
    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);
    analogWriteFreq(20000);  // 20kHz - above audible range to eliminate ESC noise
    applyPwm(0);
}

void FlashLight::update() {
    uint32_t now = millis();

    switch (_mode) {
        case FLASH_MODE_OFF:
            if (_currentPwm != 0) {
                applyPwm(0);
            }
            break;

        case FLASH_MODE_STROBE: {
            uint16_t interval = _strobeState ? _strobeOnMs : _strobeOffMs;
            if ((now - _lastUpdate) >= interval) {
                _lastUpdate = now;
                _strobeState = !_strobeState;
                applyPwm(_strobeState ? _brightness : 0);
            }
            break;
        }

        case FLASH_MODE_WALK_UP:
            if ((now - _lastUpdate) >= _walkIntervalMs) {
                _lastUpdate = now;
                applyPwm(walkValues[_walkStep]);
                _walkStep++;
                if (_walkStep >= walkSteps) {
                    _walkStep = 0;
                    // Optional: turn off briefly at the end of cycle
                    applyPwm(0);
                }
            }
            break;

        case FLASH_MODE_STEADY:
            if (_currentPwm != _brightness) {
                applyPwm(_brightness);
            }
            break;

        default:
            break;
    }
}

void FlashLight::setMode(FlashLightMode mode) {
    if (mode >= FLASH_MODE_COUNT) {
        mode = FLASH_MODE_OFF;
    }
    if (_mode != mode) {
        _mode = mode;
        _lastUpdate = millis();
        _strobeState = false;
        _walkStep = 0;

        // Immediately apply state for OFF mode
        if (_mode == FLASH_MODE_OFF) {
            applyPwm(0);
        }
    }
}

FlashLightMode FlashLight::getMode() const {
    return _mode;
}

void FlashLight::nextMode() {
    FlashLightMode next = (FlashLightMode)((_mode + 1) % FLASH_MODE_COUNT);
    setMode(next);
}

void FlashLight::setBrightness(uint8_t brightness) {
    _brightness = brightness;
}

uint8_t FlashLight::getBrightness() const {
    return _brightness;
}

void FlashLight::setStrobeInterval(uint16_t onMs, uint16_t offMs) {
    _strobeOnMs = onMs;
    _strobeOffMs = offMs;
}

void FlashLight::setWalkInterval(uint16_t intervalMs) {
    _walkIntervalMs = intervalMs;
}

void FlashLight::applyPwm(uint8_t value) {
    _currentPwm = value;
    // HW-095 ESC control:
    // IN1=PWM, IN2=LOW  -> Forward (lamp on)
    // IN1=LOW, IN2=LOW  -> Stop (lamp off)
    if (value > 0) {
        analogWrite(_pinIn1, value);
        analogWrite(_pinIn2, 0);
    } else {
        analogWrite(_pinIn1, 0);
        analogWrite(_pinIn2, 0);
    }
}
