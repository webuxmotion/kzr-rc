// CubeCell - Control Data Transmitter
// Sends control data to NodeMCU via bit-bang serial on GPIO5
// Protocol: [0xAA][0xBB][LEN][DATA...][XOR_CRC]
// Wiring: GPIO5 -> NodeMCU D1, GND -> GND

#include <Arduino.h>

#define SOFT_TX_PIN GPIO5
#define BAUD_RATE 9600
#define BIT_DELAY (1000000 / BAUD_RATE)

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0xBB

// Control data to send
int16_t txForward = 0;      // -100 to 100
int16_t txBackward = 0;     // -100 to 100
int16_t txLeft = 0;         // -100 to 100
int16_t txRight = 0;        // -100 to 100
int16_t txSpeed = 0;        // 0 to 100
int16_t txTemperature = 250; // degrees * 10 (25.0°C)

void softSerialBegin() {
    pinMode(SOFT_TX_PIN, OUTPUT);
    digitalWrite(SOFT_TX_PIN, HIGH);
}

void softSerialWrite(uint8_t b) {
    digitalWrite(SOFT_TX_PIN, LOW);
    delayMicroseconds(BIT_DELAY);

    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(SOFT_TX_PIN, (b >> i) & 1);
        delayMicroseconds(BIT_DELAY);
    }

    digitalWrite(SOFT_TX_PIN, HIGH);
    delayMicroseconds(BIT_DELAY);
}

void softSerialPrint(const uint8_t* data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        softSerialWrite(data[i]);
    }
}

void sendControlData() {
    uint8_t dataLen = 12;  // 6 values * 2 bytes
    uint8_t packet[3 + dataLen + 1];
    uint8_t idx = 0;

    // Sync bytes
    packet[idx++] = SYNC_BYTE_1;
    packet[idx++] = SYNC_BYTE_2;

    // Length
    packet[idx++] = dataLen;

    // Data (high byte first)
    packet[idx++] = (txForward >> 8) & 0xFF;
    packet[idx++] = txForward & 0xFF;

    packet[idx++] = (txBackward >> 8) & 0xFF;
    packet[idx++] = txBackward & 0xFF;

    packet[idx++] = (txLeft >> 8) & 0xFF;
    packet[idx++] = txLeft & 0xFF;

    packet[idx++] = (txRight >> 8) & 0xFF;
    packet[idx++] = txRight & 0xFF;

    packet[idx++] = (txSpeed >> 8) & 0xFF;
    packet[idx++] = txSpeed & 0xFF;

    packet[idx++] = (txTemperature >> 8) & 0xFF;
    packet[idx++] = txTemperature & 0xFF;

    // XOR checksum
    uint8_t crc = 0;
    for (uint8_t i = 2; i < idx; i++) {
        crc ^= packet[i];
    }
    packet[idx++] = crc;

    softSerialPrint(packet, idx);
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("=== CubeCell Control TX ===");
    Serial.println("TX on GPIO5 at 9600 baud");

    softSerialBegin();
}

// Test counter for demo
int16_t testValue = 0;
int16_t testDirection = 5;

void loop() {
    // === TEST: Sweep values for testing ===
    testValue += testDirection;
    if (testValue >= 100) testDirection = -5;
    if (testValue <= -100) testDirection = 5;

    // Demo: forward/backward sweep
    txForward = (testValue > 0) ? testValue : 0;
    txBackward = (testValue < 0) ? -testValue : 0;

    // Demo: left/right based on speed
    txLeft = 0;
    txRight = 0;

    // Demo: speed increases with forward
    txSpeed = abs(testValue);

    // Demo: simulated temperature
    txTemperature = 250 + (testValue / 10);  // 25.0°C ± variation

    // Send data
    sendControlData();

    // Debug output
    Serial.printf("F:%d B:%d L:%d R:%d SPD:%d TEMP:%.1f\n",
        txForward, txBackward, txLeft, txRight, txSpeed, txTemperature / 10.0);

    // === ADD YOUR SENSOR/INPUT LOGIC HERE ===
    // Example: Read joystick
    // txForward = map(analogRead(ADC), 0, 4095, 0, 100);

    // Example: Read temperature sensor
    // txTemperature = readTemperature() * 10;

    delay(20);  // ~50Hz
}
