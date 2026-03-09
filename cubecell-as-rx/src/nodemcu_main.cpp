// NodeMCU V3 - CubeCell to FC Bridge
// D1 <- CubeCell RX, TX -> FC CRSF
// D2 -> HW-095 ESC IN1 (Flash Light)
// D5 -> HW-095 ESC IN2 (Flash Light)

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AlfredoCRSF.h>
#include "flash_light.h"

#define CUBECELL_RX_PIN D1
#define FLASH_LIGHT_IN1 D2  // HW-095 ESC IN1
#define FLASH_LIGHT_IN2 D5  // HW-095 ESC IN2
#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0xBB

SoftwareSerial cubecellSerial(CUBECELL_RX_PIN, -1);
AlfredoCRSF crsf;
crsf_channels_t crsfChannels = { 0 };
FlashLight flashLight(FLASH_LIGHT_IN1, FLASH_LIGHT_IN2);

enum ParseState { WAIT_SYNC1, WAIT_SYNC2, WAIT_LEN, READ_DATA, WAIT_CRC };
ParseState parseState = WAIT_SYNC1;
uint8_t packetBuffer[32];
uint8_t packetLen = 0;
uint8_t packetIdx = 0;

int16_t rxForward = 0;
int16_t rxBackward = 0;
int16_t rxLeft = 0;
int16_t rxRight = 0;
int16_t rxSpeed = 0;
int16_t rxTemperature = 0;
uint8_t rxLampOn = 0;

uint32_t lastPacketTime = 0;
uint32_t lastCrsfTime = 0;

int16_t valueToCrsf(int16_t value) {
    return map(value, -100, 100, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
}

int16_t speedToCrsf(int16_t value) {
    return map(value, 0, 100, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
}

void processPacket() {
    if (packetLen >= 13) {
        rxForward     = (int16_t)((packetBuffer[0] << 8) | packetBuffer[1]);
        rxBackward    = (int16_t)((packetBuffer[2] << 8) | packetBuffer[3]);
        rxLeft        = (int16_t)((packetBuffer[4] << 8) | packetBuffer[5]);
        rxRight       = (int16_t)((packetBuffer[6] << 8) | packetBuffer[7]);
        rxSpeed       = (int16_t)((packetBuffer[8] << 8) | packetBuffer[9]);
        rxTemperature = (int16_t)((packetBuffer[10] << 8) | packetBuffer[11]);
        rxLampOn      = packetBuffer[12];

        crsfChannels.ch0 = valueToCrsf(rxRight - rxLeft);
        crsfChannels.ch1 = valueToCrsf(rxForward - rxBackward);
        crsfChannels.ch2 = speedToCrsf(rxSpeed);
        crsfChannels.ch3 = CRSF_CHANNEL_VALUE_MID;

        flashLight.setMode(rxLampOn ? FLASH_MODE_STEADY : FLASH_MODE_OFF);

        lastPacketTime = millis();
    }
}

void parseByte(uint8_t b) {
    static uint8_t xorCrc = 0;

    switch (parseState) {
        case WAIT_SYNC1:
            if (b == SYNC_BYTE_1) parseState = WAIT_SYNC2;
            break;
        case WAIT_SYNC2:
            parseState = (b == SYNC_BYTE_2) ? WAIT_LEN : WAIT_SYNC1;
            break;
        case WAIT_LEN:
            if (b > 0 && b <= 32) {
                packetLen = b;
                packetIdx = 0;
                xorCrc = b;
                parseState = READ_DATA;
            } else {
                parseState = WAIT_SYNC1;
            }
            break;
        case READ_DATA:
            packetBuffer[packetIdx++] = b;
            xorCrc ^= b;
            if (packetIdx >= packetLen) parseState = WAIT_CRC;
            break;
        case WAIT_CRC:
            if (b == xorCrc) processPacket();
            parseState = WAIT_SYNC1;
            break;
    }
}

void setup() {
    Serial.begin(420000);
    crsf.begin(Serial);
    cubecellSerial.begin(9600);

    flashLight.begin();
    flashLight.setMode(FLASH_MODE_OFF);  // Controlled by TX

    crsfChannels.ch0 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch1 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch2 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch3 = CRSF_CHANNEL_VALUE_MID;
    crsfChannels.ch4 = CRSF_CHANNEL_VALUE_MIN;
    crsfChannels.ch5 = CRSF_CHANNEL_VALUE_MIN;
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
    while (cubecellSerial.available()) parseByte(cubecellSerial.read());

    if ((millis() - lastPacketTime) > 500) {
        crsfChannels.ch0 = CRSF_CHANNEL_VALUE_MID;
        crsfChannels.ch1 = CRSF_CHANNEL_VALUE_MID;
        crsfChannels.ch2 = CRSF_CHANNEL_VALUE_MIN;
        flashLight.setMode(FLASH_MODE_OFF);
    }

    if ((millis() - lastCrsfTime) >= 20) {
        lastCrsfTime = millis();
        crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));
    }

    flashLight.update();

    yield();
}
