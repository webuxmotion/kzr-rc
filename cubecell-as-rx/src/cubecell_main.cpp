// CubeCell - Software Serial TX
// Sends data to NodeMCU via bit-bang serial on GPIO5 (SCL pin)
// Wiring: GPIO5 -> D1, GND -> GND

#include <Arduino.h>

#define SOFT_TX_PIN GPIO5  // Using SCL pin for serial TX
#define BAUD_RATE 9600
#define BIT_DELAY (1000000 / BAUD_RATE)  // microseconds per bit

void softSerialBegin() {
    pinMode(SOFT_TX_PIN, OUTPUT);
    digitalWrite(SOFT_TX_PIN, HIGH);  // Idle state
}

void softSerialWrite(uint8_t b) {
    // Start bit
    digitalWrite(SOFT_TX_PIN, LOW);
    delayMicroseconds(BIT_DELAY);

    // Data bits (LSB first)
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(SOFT_TX_PIN, (b >> i) & 1);
        delayMicroseconds(BIT_DELAY);
    }

    // Stop bit
    digitalWrite(SOFT_TX_PIN, HIGH);
    delayMicroseconds(BIT_DELAY);
}

void softSerialPrint(const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        softSerialWrite(data[i]);
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("=== CubeCell Soft Serial TX ===");
    Serial.println("TX on GPIO5 at 9600 baud");

    softSerialBegin();
}

void loop() {
    static uint8_t counter = 0;
    uint8_t testData[] = {0xAA, 0xBB, counter++};

    // Send via soft serial
    softSerialPrint(testData, sizeof(testData));

    // Debug output
    Serial.print("Sent: ");
    for (uint8_t i = 0; i < sizeof(testData); i++) {
        Serial.printf("%02X ", testData[i]);
    }
    Serial.println();

    delay(2000);
}
