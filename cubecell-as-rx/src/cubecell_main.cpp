// CubeCell RX - LoRa Receiver to NodeMCU Bridge
// Receives LoRa data from TX CubeCell, forwards to NodeMCU
// Wiring: GPIO5 -> NodeMCU D1, GND -> GND

#include <Arduino.h>
#include "LoRaWan_APP.h"

// LoRa settings - must match TX CubeCell
#define RF_FREQUENCY 433000000  // Hz
#define TX_OUTPUT_POWER 14      // dBm
#define LORA_BANDWIDTH 0        // 0: 125kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1       // 4/5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 64

// Software Serial to NodeMCU
#define SOFT_TX_PIN GPIO5
#define BAUD_RATE 9600
#define BIT_DELAY (1000000 / BAUD_RATE)

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0xBB

// LoRa state
static RadioEvents_t RadioEvents;
char rxBuffer[BUFFER_SIZE];
int16_t rxRssi;
int8_t rxSnr;
bool loraDataReceived = false;

// Control data received from TX CubeCell
int16_t rxForward = 0;
int16_t rxBackward = 0;
int16_t rxLeft = 0;
int16_t rxRight = 0;
int16_t rxSpeed = 0;
int16_t rxTemperature = 0;

uint32_t lastRxTime = 0;

// Software Serial functions
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

// Send data to NodeMCU
void sendToNodeMCU() {
    uint8_t dataLen = 12;  // 6 values * 2 bytes
    uint8_t packet[3 + dataLen + 1];
    uint8_t idx = 0;

    packet[idx++] = SYNC_BYTE_1;
    packet[idx++] = SYNC_BYTE_2;
    packet[idx++] = dataLen;

    // Data (high byte first)
    packet[idx++] = (rxForward >> 8) & 0xFF;
    packet[idx++] = rxForward & 0xFF;
    packet[idx++] = (rxBackward >> 8) & 0xFF;
    packet[idx++] = rxBackward & 0xFF;
    packet[idx++] = (rxLeft >> 8) & 0xFF;
    packet[idx++] = rxLeft & 0xFF;
    packet[idx++] = (rxRight >> 8) & 0xFF;
    packet[idx++] = rxRight & 0xFF;
    packet[idx++] = (rxSpeed >> 8) & 0xFF;
    packet[idx++] = rxSpeed & 0xFF;
    packet[idx++] = (rxTemperature >> 8) & 0xFF;
    packet[idx++] = rxTemperature & 0xFF;

    // XOR checksum
    uint8_t crc = 0;
    for (uint8_t i = 2; i < idx; i++) {
        crc ^= packet[i];
    }
    packet[idx++] = crc;

    softSerialPrint(packet, idx);
}

// LoRa RX callback
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    rxRssi = rssi;
    rxSnr = snr;

    if (size >= 12) {
        // Parse received data (high byte first)
        rxForward     = (int16_t)((payload[0] << 8) | payload[1]);
        rxBackward    = (int16_t)((payload[2] << 8) | payload[3]);
        rxLeft        = (int16_t)((payload[4] << 8) | payload[5]);
        rxRight       = (int16_t)((payload[6] << 8) | payload[7]);
        rxSpeed       = (int16_t)((payload[8] << 8) | payload[9]);
        rxTemperature = (int16_t)((payload[10] << 8) | payload[11]);

        loraDataReceived = true;
        lastRxTime = millis();
    }

    // Continue listening
    Radio.Rx(0);
}

void OnRxTimeout() {
    Radio.Rx(0);
}

void OnRxError() {
    Radio.Rx(0);
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("=== CubeCell LoRa RX ===");
    Serial.printf("Freq: %d Hz\n", RF_FREQUENCY);

    softSerialBegin();

    // Initialize LoRa
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(
        MODEM_LORA,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        0,  // AFC bandwidth (not used for LoRa)
        LORA_PREAMBLE_LENGTH,
        0,  // Symbol timeout
        LORA_FIX_LENGTH_PAYLOAD_ON,
        0,  // Payload length (0 = variable)
        true,  // CRC on
        0, 0,  // Freq hop
        LORA_IQ_INVERSION_ON,
        true   // RX continuous
    );

    // Start receiving
    Radio.Rx(0);
    Serial.println("Listening for LoRa...");
}

void loop() {
    // Process LoRa events
    Radio.IrqProcess();

    // Forward data to NodeMCU when received
    if (loraDataReceived) {
        loraDataReceived = false;
        sendToNodeMCU();

        Serial.printf("RX: F:%d B:%d L:%d R:%d SPD:%d T:%.1f RSSI:%d SNR:%d\n",
            rxForward, rxBackward, rxLeft, rxRight, rxSpeed,
            rxTemperature / 10.0, rxRssi, rxSnr);
    }

    // Failsafe: if no data for 500ms, send zeros
    if ((millis() - lastRxTime) > 500 && lastRxTime > 0) {
        rxForward = 0;
        rxBackward = 0;
        rxLeft = 0;
        rxRight = 0;
        rxSpeed = 0;
        sendToNodeMCU();
        lastRxTime = millis();  // Reset to avoid spamming
        Serial.println("FAILSAFE: No LoRa data");
    }
}
