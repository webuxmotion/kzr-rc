// ESP32-C3 Super Mini - Buttons to CubeCell TX Bridge
// 1-Wire pulse protocol on GPIO10 -> CubeCell GPIO5
// Buttons: GPIO3, GPIO4

#include <Arduino.h>

#define DATA_PIN 10
#define BUTTON1_PIN 3
#define BUTTON2_PIN 4
#define BUTTON3_PIN 5  // Lamp button

// Pulse timing (microseconds)
#define START_PULSE 3000   // Start pulse LOW duration
#define START_GAP   2000   // Gap after start (increased)
#define BIT_0_PULSE 400    // Short pulse = 0
#define BIT_1_PULSE 1600   // Long pulse = 1
#define BIT_GAP     800    // Gap between bits (increased)

uint8_t button1 = 0;
uint8_t button2 = 0;
uint8_t button3 = 0;  // Lamp

uint32_t lastTxTime = 0;

void sendBit(uint8_t bit) {
    digitalWrite(DATA_PIN, LOW);
    delayMicroseconds(bit ? BIT_1_PULSE : BIT_0_PULSE);
    digitalWrite(DATA_PIN, HIGH);
    delayMicroseconds(BIT_GAP);
}

void sendPacket(uint8_t btn1, uint8_t btn2, uint8_t btn3) {
    // Start pulse
    digitalWrite(DATA_PIN, LOW);
    delayMicroseconds(START_PULSE);
    digitalWrite(DATA_PIN, HIGH);
    delayMicroseconds(START_GAP);

    // Send button states
    sendBit(btn1);
    sendBit(btn2);
    sendBit(btn3);

    // Send parity bit (XOR of all buttons)
    sendBit(btn1 ^ btn2 ^ btn3);
}

void readButtons() {
    button1 = (digitalRead(BUTTON1_PIN) == LOW) ? 1 : 0;
    button2 = (digitalRead(BUTTON2_PIN) == LOW) ? 1 : 0;
    button3 = (digitalRead(BUTTON3_PIN) == LOW) ? 1 : 0;
}

void setup() {
    Serial.begin(115200);

    pinMode(DATA_PIN, OUTPUT);
    digitalWrite(DATA_PIN, HIGH);  // Idle HIGH

    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);

    Serial.println("ESP32-C3 Buttons Ready (Pulse Protocol)");
}

void loop() {
    if ((millis() - lastTxTime) >= 50) {
        lastTxTime = millis();
        readButtons();
        sendPacket(button1, button2, button3);
        Serial.printf("btn1=%d btn2=%d btn3=%d\n", button1, button2, button3);
    }
}
