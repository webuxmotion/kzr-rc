// CubeCell TX - LoRa Transmitter with Direct Button Input
// 6 buttons on GPIO0-GPIO5

#include <Arduino.h>
#include "LoRaWan_APP.h"

// Button pin definitions
const uint8_t BUTTON_PINS[] = {GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5};
const uint8_t NUM_BUTTONS = 6;

// Debounce configuration
const unsigned long DEBOUNCE_DELAY = 50;

#define RF_FREQUENCY 433000000
#define TX_OUTPUT_POWER 14
#define LORA_BANDWIDTH 1              // 0=125kHz, 1=250kHz (faster)
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8

static RadioEvents_t RadioEvents;
bool txDone = true;

// Button state tracking
struct Button {
    uint8_t pin;
    bool lastState;
    bool currentState;
    unsigned long lastDebounceTime;
};

Button buttons[NUM_BUTTONS];

// TX values
int16_t txForward = 0;
int16_t txBackward = 0;
int16_t txLeft = 0;
int16_t txRight = 0;
int16_t txSpeed = 0;
int16_t txTemperature = 250;
uint8_t txLampOn = 0;

uint32_t lastTxTime = 0;

void OnTxDone() {
    txDone = true;
    Radio.Sleep();
    Serial.println("TX Done");
}

void OnTxTimeout() {
    txDone = true;
    Radio.Sleep();
    Serial.println("TX Timeout!");
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

    // Debug transmitted data
    Serial.print("TX: Fwd=");
    Serial.print(txForward);
    Serial.print(" Bwd=");
    Serial.print(txBackward);
    Serial.print(" L=");
    Serial.print(txLeft);
    Serial.print(" R=");
    Serial.print(txRight);
    Serial.print(" Spd=");
    Serial.print(txSpeed);
    Serial.print(" Temp=");
    Serial.print(txTemperature);
    Serial.print(" Lamp=");
    Serial.println(txLampOn);
}

void updateButtonStates() {
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        bool reading = digitalRead(buttons[i].pin);

        if (reading != buttons[i].lastState) {
            buttons[i].lastDebounceTime = millis();
        }

        if ((millis() - buttons[i].lastDebounceTime) > DEBOUNCE_DELAY) {
            buttons[i].currentState = reading;
        }

        buttons[i].lastState = reading;
    }

    // Map buttons to controls (LOW = pressed with INPUT_PULLUP)
    // Button 0: Forward
    // Button 1: Backward
    // Button 2: Left
    // Button 3: Right
    // Button 4: Speed
    // Button 5: Lamp toggle
    bool btnForward = (buttons[0].currentState == LOW);
    bool btnBackward = (buttons[1].currentState == LOW);
    bool btnLeft = (buttons[2].currentState == LOW);
    bool btnRight = (buttons[3].currentState == LOW);
    bool btnSpeed = (buttons[4].currentState == LOW);
    bool btnLamp = (buttons[5].currentState == LOW);

    txForward = btnForward ? 100 : 0;
    txBackward = btnBackward ? 100 : 0;
    txLeft = btnLeft ? 100 : 0;
    txRight = btnRight ? 100 : 0;
    txSpeed = (btnForward || btnBackward || btnLeft || btnRight) ? 100 : 0;
    if (btnSpeed) txSpeed = 100;
    txLampOn = btnLamp ? 1 : 0;

    // Debug button presses
    static bool lastBtnStates[6] = {false};
    const char* btnNames[] = {"Forward", "Backward", "Left", "Right", "Speed", "Lamp"};
    bool currentStates[] = {btnForward, btnBackward, btnLeft, btnRight, btnSpeed, btnLamp};

    for (uint8_t i = 0; i < 6; i++) {
        if (currentStates[i] != lastBtnStates[i]) {
            Serial.print("Button ");
            Serial.print(btnNames[i]);
            Serial.println(currentStates[i] ? " PRESSED" : " RELEASED");
            lastBtnStates[i] = currentStates[i];
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("CubeCell TX - LoRa Transmitter Starting...");

    // Initialize buttons
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
        buttons[i].pin = BUTTON_PINS[i];
        buttons[i].lastState = HIGH;
        buttons[i].currentState = HIGH;
        buttons[i].lastDebounceTime = 0;
        pinMode(buttons[i].pin, INPUT_PULLUP);
    }

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
        false, true, 0, 0, false, 3000);

    Serial.println("LoRa initialized!");
    Serial.print("Frequency: ");
    Serial.print(RF_FREQUENCY / 1000000.0);
    Serial.println(" MHz");
    Serial.println("Ready to transmit...");
}

void loop() {
    Radio.IrqProcess();

    updateButtonStates();

    // Send LoRa packet every 30ms
    if ((millis() - lastTxTime) >= 30 && txDone) {
        lastTxTime = millis();
        sendLoRaPacket();
    }
}
