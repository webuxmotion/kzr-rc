// NodeMCU V3 - CRSF to Flight Controller using AlfredoCRSF
// Sends CRSF RC channel data directly to FC
// Wiring: TX (GPIO1) -> FC CRSF RX, GND -> GND

#include <Arduino.h>
#include <AlfredoCRSF.h>

AlfredoCRSF crsf;

// Channel values (CRSF range: 172-1811)
crsf_channels_t crsfChannels = { 0 };

uint32_t lastCrsfTime = 0;
int16_t testValue = CRSF_CHANNEL_VALUE_MIN;
int16_t direction = 10;  // Increment step

void sendChannels() {
    crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));
}

void updateTestValues() {
    // Sweep value between MIN and MAX
    testValue += direction;
    if (testValue >= CRSF_CHANNEL_VALUE_MAX) {
        testValue = CRSF_CHANNEL_VALUE_MAX;
        direction = -10;
    } else if (testValue <= CRSF_CHANNEL_VALUE_MIN) {
        testValue = CRSF_CHANNEL_VALUE_MIN;
        direction = 10;
    }

    // Apply to channels (Roll, Pitch, Yaw sweep; Throttle stays low for safety)
    crsfChannels.ch0 = testValue;  // Roll
    crsfChannels.ch1 = testValue;  // Pitch
    crsfChannels.ch3 = testValue;  // Yaw
    // ch2 (Throttle) stays at MIN for safety
}

void setup() {
    // CRSF baud rate - check your FC settings in Betaflight
    // Common options: 420000, 400000, 250000, 115200
    Serial.begin(420000);
    crsf.begin(Serial);
    delay(100);

    // Set default channel values (center sticks, low throttle)
    crsfChannels.ch0 = CRSF_CHANNEL_VALUE_MIN;   // Roll
    crsfChannels.ch1 = CRSF_CHANNEL_VALUE_MIN;   // Pitch
    crsfChannels.ch2 = CRSF_CHANNEL_VALUE_MIN;   // Throttle
    crsfChannels.ch3 = CRSF_CHANNEL_VALUE_MIN;   // Yaw
    crsfChannels.ch4 = CRSF_CHANNEL_VALUE_MIN;   // Aux1 (Arm)
    crsfChannels.ch5 = CRSF_CHANNEL_VALUE_MIN;   // Aux2
    crsfChannels.ch6 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch7 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch8 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch9 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch10 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch11 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch12 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch13 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch14 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch15 = CRSF_CHANNEL_VALUE_MIN;
}

void loop() {
    // Send CRSF frames at ~50Hz (every 20ms)
    if ((millis() - lastCrsfTime) >= 20) {
        lastCrsfTime = millis();
        updateTestValues();
        sendChannels();
    }

    yield();
}
