// CubeCell RX - LoRa to NodeMCU Bridge
// GPIO5 -> NodeMCU D1

#include <Arduino.h>
#include "LoRaWan_APP.h"

#define SOFT_TX_PIN GPIO5
#define BAUD_RATE 9600
#define BIT_DELAY (1000000 / BAUD_RATE)

#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0xBB

#define RF_FREQUENCY 433000000
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8

static RadioEvents_t RadioEvents;
int16_t rxRssi;
bool loraDataReceived = false;

int16_t rxForward = 0;
int16_t rxBackward = 0;
int16_t rxLeft = 0;
int16_t rxRight = 0;
int16_t rxSpeed = 0;
int16_t rxTemperature = 0;

uint32_t lastRxTime = 0;

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

void sendToNodeMCU() {
    uint8_t packet[16];
    uint8_t idx = 0;

    packet[idx++] = SYNC_BYTE_1;
    packet[idx++] = SYNC_BYTE_2;
    packet[idx++] = 12;

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

    uint8_t crc = 0;
    for (uint8_t i = 2; i < idx; i++) crc ^= packet[i];
    packet[idx++] = crc;

    for (uint8_t i = 0; i < idx; i++) softSerialWrite(packet[i]);
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    rxRssi = rssi;
    if (size >= 12) {
        rxForward     = (int16_t)((payload[0] << 8) | payload[1]);
        rxBackward    = (int16_t)((payload[2] << 8) | payload[3]);
        rxLeft        = (int16_t)((payload[4] << 8) | payload[5]);
        rxRight       = (int16_t)((payload[6] << 8) | payload[7]);
        rxSpeed       = (int16_t)((payload[8] << 8) | payload[9]);
        rxTemperature = (int16_t)((payload[10] << 8) | payload[11]);
        loraDataReceived = true;
        lastRxTime = millis();
    }
    Radio.Rx(0);
}

void OnRxTimeout() { Radio.Rx(0); }
void OnRxError() { Radio.Rx(0); }

void setup() {
    Serial.begin(115200);

    pinMode(SOFT_TX_PIN, OUTPUT);
    digitalWrite(SOFT_TX_PIN, HIGH);

    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
        LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, 0, false, 0, true, 0, 0, false, true);

    Radio.Rx(0);
    Serial.println("CubeCell RX Ready");
}

void loop() {
    Radio.IrqProcess();

    if (loraDataReceived) {
        loraDataReceived = false;
        sendToNodeMCU();
    }

    if ((millis() - lastRxTime) > 500 && lastRxTime > 0) {
        rxForward = rxBackward = rxLeft = rxRight = rxSpeed = 0;
        sendToNodeMCU();
        lastRxTime = millis();
    }
}
