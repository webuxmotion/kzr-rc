// CubeCell TX - LoRa Transmitter
// Buttons: GPIO1, GPIO2

#include <Arduino.h>
#include "LoRaWan_APP.h"

#define BUTTON1_PIN GPIO1
#define BUTTON2_PIN GPIO2

#define RF_FREQUENCY 433000000
#define TX_OUTPUT_POWER 14
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8

static RadioEvents_t RadioEvents;
bool txDone = true;

bool button1Pressed = false;
bool button2Pressed = false;

int16_t txForward = 0;
int16_t txBackward = 0;
int16_t txLeft = 0;
int16_t txRight = 0;
int16_t txSpeed = 0;
int16_t txTemperature = 250;

uint32_t lastTxTime = 0;

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

    uint8_t packet[12];
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

    txDone = false;
    Radio.Send(packet, sizeof(packet));
}

void readButtons() {
    button1Pressed = (digitalRead(BUTTON1_PIN) == LOW);
    button2Pressed = (digitalRead(BUTTON2_PIN) == LOW);

    txForward = button1Pressed ? 100 : 0;
    txBackward = button2Pressed ? 100 : 0;
    txSpeed = (button1Pressed || button2Pressed) ? 100 : 0;
    txLeft = 0;
    txRight = 0;
}

void setup() {
    Serial.begin(115200);

    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
        false, true, 0, 0, false, 3000);

    Serial.println("CubeCell TX Ready");
}

void loop() {
    Radio.IrqProcess();

    if ((millis() - lastTxTime) >= 20 && txDone) {
        lastTxTime = millis();
        readButtons();
        sendLoRaPacket();
    }
}
