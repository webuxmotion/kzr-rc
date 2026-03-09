// NodeMCU V3 - Software Serial RX
// Receives data from CubeCell via SoftwareSerial on D1
// Wiring: D1 (RX) -> CubeCell GPIO5, GND -> GND

#include <Arduino.h>
#include <SoftwareSerial.h>

#define SOFT_RX_PIN D1  // Receive from CubeCell
#define SOFT_TX_PIN D2  // Not used, but required

SoftwareSerial softSerial(SOFT_RX_PIN, SOFT_TX_PIN);

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("\n=== NodeMCU Soft Serial RX ===");
    Serial.println("RX on D1 at 9600 baud");
    Serial.println("Waiting for data from CubeCell...");

    softSerial.begin(9600);
}

void loop() {
    if (softSerial.available()) {
        Serial.print("Received: ");
        while (softSerial.available()) {
            uint8_t b = softSerial.read();
            Serial.printf("%02X ", b);
        }
        Serial.println();
    }
    yield();
}
