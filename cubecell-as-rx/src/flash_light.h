// Flash Light Module for NodeMCU V3
// PWM-based lamp control with multiple modes
// Supports HW-095 ESC (2-pin control: IN1, IN2)

#ifndef FLASH_LIGHT_H
#define FLASH_LIGHT_H

#include <Arduino.h>

// Light modes
enum FlashLightMode {
    FLASH_MODE_OFF = 0,
    FLASH_MODE_STROBE,
    FLASH_MODE_WALK_UP,
    FLASH_MODE_STEADY,
    FLASH_MODE_COUNT
};

class FlashLight {
public:
    // Two-pin constructor for HW-095 ESC (IN1, IN2)
    FlashLight(uint8_t pinIn1, uint8_t pinIn2);

    void begin();
    void update();  // Call this in loop() - non-blocking

    void setMode(FlashLightMode mode);
    FlashLightMode getMode() const;
    void nextMode();

    void setBrightness(uint8_t brightness);  // 0-255 for steady mode
    uint8_t getBrightness() const;

    void setStrobeInterval(uint16_t onMs, uint16_t offMs);
    void setWalkInterval(uint16_t intervalMs);

private:
    uint8_t _pinIn1;
    uint8_t _pinIn2;
    FlashLightMode _mode;
    uint8_t _brightness;
    uint8_t _currentPwm;

    // Timing
    uint32_t _lastUpdate;
    uint16_t _strobeOnMs;
    uint16_t _strobeOffMs;
    uint16_t _walkIntervalMs;

    // State
    bool _strobeState;
    uint8_t _walkStep;

    // HW-095 control: speed > 0 drives IN1, speed = 0 stops both
    void applyPwm(uint8_t value);
};

#endif // FLASH_LIGHT_H
