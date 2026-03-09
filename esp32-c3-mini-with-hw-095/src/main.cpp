/*
 * ESP32-C3 Super Mini + HW-095 ESC Motor Test
 *
 * Spins motor from 0 to max speed, then reverses, infinitely.
 *
 * WIRING DIAGRAM:
 * ===============
 *
 *   ESP32-C3 Super Mini          HW-095 ESC
 *   ==================          ===========
 *        GPIO2  -----------------> IN1 (PWM input 1)
 *        GPIO3  -----------------> IN2 (PWM input 2)
 *        GND    -----------------> GND
 *
 *                                  HW-095 ESC
 *                                 ===========
 *   Battery (+) -----------------> VCC (Motor power, 6-27V)
 *   Battery (-) -----------------> GND
 *
 *                                  HW-095 ESC
 *                                 ===========
 *   Motor Wire 1 <----------------- OUT1
 *   Motor Wire 2 <----------------- OUT2
 *
 *
 *   ASCII Wiring Diagram:
 *   =====================
 *
 *   +-----------------+          +-------------+          +-------+
 *   | ESP32-C3 Mini   |          |  HW-095 ESC |          | Motor |
 *   |                 |          |             |          |       |
 *   |  GPIO2 o--------+--------->| IN1         |          |       |
 *   |  GPIO3 o--------+--------->| IN2         |          |       |
 *   |    GND o--------+--------->| GND    OUT1 |--------->| M+    |
 *   |                 |    +---->| VCC    OUT2 |--------->| M-    |
 *   +-----------------+    |     +-------------+          +-------+
 *                          |
 *   +-----------------+    |
 *   | Battery 6-27V   |    |
 *   |  (+) o----------+----+
 *   |  (-) o-----------------> GND (common ground)
 *   +-----------------+
 *
 *
 * HW-095 Control Logic:
 * - IN1=HIGH, IN2=LOW  -> Motor Forward
 * - IN1=LOW,  IN2=HIGH -> Motor Reverse
 * - IN1=LOW,  IN2=LOW  -> Motor Stop (coast)
 * - IN1=HIGH, IN2=HIGH -> Motor Brake
 *
 * PWM on IN1/IN2 controls speed (0-255)
 */

#include <Arduino.h>

// Pin definitions
#define IN1_PIN 2   // PWM output to ESC IN1
#define IN2_PIN 3   // PWM output to ESC IN2

// PWM configuration
#define PWM_FREQ 25000    // 25kHz - above audible range, no noise
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255)
#define PWM_CHANNEL_1 0   // LEDC channel for IN1
#define PWM_CHANNEL_2 1   // LEDC channel for IN2

// Speed ramp settings
#define RAMP_DELAY_MS 10  // Delay between speed steps (faster for smoother visual)
#define MAX_SPEED 255
#define MIN_SPEED 0

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-C3 + HW-095 ESC Motor Test");
  Serial.println("================================");

  // Configure PWM channels
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);

  // Attach channels to pins
  ledcAttachPin(IN1_PIN, PWM_CHANNEL_1);
  ledcAttachPin(IN2_PIN, PWM_CHANNEL_2);

  // Start with motor stopped
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);

  Serial.println("Motor initialized. Starting test...");
  delay(2000);
}

void setMotorSpeed(int speed) {
  // speed: -255 to +255
  // Positive = forward, Negative = reverse

  if (speed > 0) {
    // Forward
    ledcWrite(PWM_CHANNEL_1, speed);
    ledcWrite(PWM_CHANNEL_2, 0);
  } else if (speed < 0) {
    // Reverse
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, -speed);
  } else {
    // Stop
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, 0);
  }
}

// Mode definitions
#define MODE_STROBE 0
#define MODE_WALK_UP 1
#define NUM_MODES 2

int currentMode = MODE_WALK_UP;

// Walk up brightness values
const int walkValues[] = {10, 40, 60, 100, 150, 200, 255};
const int walkSteps = sizeof(walkValues) / sizeof(walkValues[0]);

void modeStrobe() {
  // Blink ON
  setMotorSpeed(MAX_SPEED);
  Serial.println("Strobe: ON");
  delay(200);

  // Blink OFF
  setMotorSpeed(0);
  Serial.println("Strobe: OFF");
  delay(200);
}

void modeWalkUp() {
  for (int i = 0; i < walkSteps; i++) {
    setMotorSpeed(walkValues[i]);
    Serial.printf("Walk up: %d\n", walkValues[i]);
    delay(1000);
  }
  // Turn off after reaching max
  setMotorSpeed(0);
  Serial.println("Walk up: OFF");
  delay(1000);
}

void loop() {
  switch (currentMode) {
    case MODE_STROBE:
      modeStrobe();
      break;
    case MODE_WALK_UP:
      modeWalkUp();
      break;
  }
}
