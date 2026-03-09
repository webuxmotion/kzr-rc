// CubeCell TX - LoRa Transmitter
// Sends control data via LoRa to RX CubeCell
// Buttons on GPIO1 and GPIO2

#include <Arduino.h>
#include "LoRaWan_APP.h"

// Button pins
#define BUTTON1_PIN GPIO1
#define BUTTON2_PIN GPIO2

// LoRa settings - must match RX CubeCell
#define RF_FREQUENCY 433000000  // Hz
#define TX_OUTPUT_POWER 14      // dBm
#define LORA_BANDWIDTH 0        // 0: 125kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1       // 4/5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

// LoRa state
static RadioEvents_t RadioEvents;
bool txDone = true;

// Button states
bool button1Pressed = false;
bool button2Pressed = false;

// Control data to send
int16_t txForward = 0;      // 0 to 100
int16_t txBackward = 0;     // 0 to 100
int16_t txLeft = 0;         // 0 to 100
int16_t txRight = 0;        // 0 to 100
int16_t txSpeed = 0;        // 0 to 100
int16_t txTemperature = 250; // degrees * 10

uint32_t lastTxTime = 0;
uint32_t txCount = 0;

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

    // Pack data (high byte first)
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
    txCount++;
}

void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("=== CubeCell LoRa TX ===");
    Serial.printf("Freq: %d Hz\n", RF_FREQUENCY);

    // Initialize buttons (active LOW with internal pullup)
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);

    // Initialize LoRa
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(
        MODEM_LORA,
        TX_OUTPUT_POWER,
        0,  // FSK deviation (not used)
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true,  // CRC on
        0, 0,  // Freq hop
        LORA_IQ_INVERSION_ON,
        3000   // TX timeout ms
    );

    Serial.println("Ready to transmit");
    Serial.println("Button1: GPIO1, Button2: GPIO2");
}

void readButtons() {
    // Buttons are active LOW (pressed = LOW, released = HIGH)
    button1Pressed = (digitalRead(BUTTON1_PIN) == LOW);
    button2Pressed = (digitalRead(BUTTON2_PIN) == LOW);

    // Button1 = Forward, Button2 = Backward
    txForward = button1Pressed ? 100 : 0;
    txBackward = button2Pressed ? 100 : 0;

    // Speed based on any button pressed
    if (button1Pressed || button2Pressed) {
        txSpeed = 100;
    } else {
        txSpeed = 0;
    }

    // Left/Right not used with buttons
    txLeft = 0;
    txRight = 0;
}

void loop() {
    // Process radio events
    Radio.IrqProcess();

    // Send at ~50Hz
    if ((millis() - lastTxTime) >= 20 && txDone) {
        lastTxTime = millis();

        // Read button inputs
        readButtons();

        sendLoRaPacket();

        // Debug (every 500ms)
        if (txCount % 25 == 0) {
            Serial.printf("TX #%lu: B1:%d B2:%d F:%d B:%d SPD:%d\n",
                txCount, button1Pressed, button2Pressed,
                txForward, txBackward, txSpeed);
        }
    }
}
