// CubeCell TX - LoRa Transmitter
// 1-Wire pulse protocol on GPIO5 <- ESP32-C3 GPIO10

#include <Arduino.h>
#include "LoRaWan_APP.h"

#define DATA_PIN GPIO5

// Pulse timing thresholds (microseconds)
#define START_MIN 2000     // Start pulse > 2ms
#define BIT_THRESHOLD 800  // < 0.8ms = 0, > 0.8ms = 1
#define MIN_PULSE 200      // Ignore pulses shorter than 0.2ms (noise)

#define RF_FREQUENCY 433000000
#define TX_OUTPUT_POWER 14
#define LORA_BANDWIDTH 1              // 0=125kHz, 1=250kHz (faster)
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8

static RadioEvents_t RadioEvents;
bool txDone = true;

uint8_t txLampOn = 0;
uint8_t btn1 = 0;
uint8_t btn2 = 0;
uint8_t btn3 = 0;  // Lamp button

int16_t txForward = 0;
int16_t txBackward = 0;
int16_t txLeft = 0;
int16_t txRight = 0;
int16_t txSpeed = 0;
int16_t txTemperature = 250;

uint32_t lastTxTime = 0;
uint32_t lastRxTime = 0;

// Measure LOW pulse duration
uint32_t measurePulse() {
    uint32_t start = micros();
    while (digitalRead(DATA_PIN) == LOW) {
        if ((micros() - start) > 10000) return 0;  // Timeout
    }
    return micros() - start;
}

// Read one bit (wait for LOW, measure duration)
int8_t readBit() {
    // Wait for LOW (start of bit pulse)
    uint32_t waitStart = micros();
    while (digitalRead(DATA_PIN) == HIGH) {
        if ((micros() - waitStart) > 5000) return -1;  // Timeout
    }

    uint32_t pulseLen = measurePulse();
    if (pulseLen == 0) return -1;
    if (pulseLen < MIN_PULSE) return -1;  // Reject noise

    return (pulseLen > BIT_THRESHOLD) ? 1 : 0;
}

// Try to receive a packet
bool receivePacket() {
    // Check if line is LOW (potential start pulse)
    if (digitalRead(DATA_PIN) == HIGH) return false;

    uint32_t pulseLen = measurePulse();

    // Check for start pulse (> 2ms)
    if (pulseLen < START_MIN) return false;

    // Wait for gap to end
    uint32_t gapStart = micros();
    while (digitalRead(DATA_PIN) == HIGH) {
        if ((micros() - gapStart) > 3000) return false;
    }

    // Read button 1
    int8_t b1 = readBit();
    if (b1 < 0) return false;

    // Read button 2
    int8_t b2 = readBit();
    if (b2 < 0) return false;

    // Read button 3 (lamp)
    int8_t b3 = readBit();
    if (b3 < 0) return false;

    // Read parity bit
    int8_t parity = readBit();
    if (parity < 0) return false;

    // Verify parity (XOR of all buttons should equal parity bit)
    if ((b1 ^ b2 ^ b3) != parity) return false;

    btn1 = b1;
    btn2 = b2;
    btn3 = b3;

    txForward = btn1 ? 100 : 0;
    txBackward = btn2 ? 100 : 0;
    txLeft = 0;
    txRight = 0;
    txSpeed = (btn1 || btn2) ? 100 : 0;
    txLampOn = btn3;

    lastRxTime = millis();
    return true;
}

void OnTxDone() {
    txDone = true;
    Radio.Sleep();
}

void OnTxTimeout() {
    txDone = true;
    Radio.Sleep();
}

void sendLoRaPacket() {
    if (!txDone) return;

    uint8_t packet[13];
    packet[0] = (txForward >> 8) & 0xFF;
    packet[1] = txForward & 0xFF;
    packet[2] = (txBackward >> 8) & 0xFF;
    packet[3] = txBackward & 0xFF;
    packet[4] = (txLeft >> 8) & 0xFF;
    packet[5] = txLeft & 0xFF;
    packet[6] = (txRight >> 8) & 0xFF;
    packet[7] = txRight & 0xFF;
    packet[8] = (txSpeed >> 8) & 0xFF;
    packet[9] = txSpeed & 0xFF;
    packet[10] = (txTemperature >> 8) & 0xFF;
    packet[11] = txTemperature & 0xFF;
    packet[12] = txLampOn;

    txDone = false;
    Radio.Send(packet, sizeof(packet));
}

void setup() {
    pinMode(DATA_PIN, INPUT_PULLUP);

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
        false, true, 0, 0, false, 3000);
}

void loop() {
    Radio.IrqProcess();

    // Try to receive pulse packet
    receivePacket();

    // Send LoRa packet every 30ms
    if ((millis() - lastTxTime) >= 30 && txDone) {
        lastTxTime = millis();
        sendLoRaPacket();
    }

    // Failsafe: zero values if no data for 500ms
    if (lastRxTime > 0 && (millis() - lastRxTime) > 500) {
        txForward = txBackward = txLeft = txRight = txSpeed = 0;
        btn1 = btn2 = 0;
    }
}
