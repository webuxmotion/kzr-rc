// NodeMCU V3 - CubeCell to FC Bridge
// Receives data from CubeCell, sends CRSF to Flight Controller
// Wiring:
//   D1 <- CubeCell GPIO5 (9600 baud)
//   TX (GPIO1) -> FC CRSF RX (420000 baud)
//   GND -> GND

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AlfredoCRSF.h>

// CubeCell SoftwareSerial
#define CUBECELL_RX_PIN D1
SoftwareSerial cubecellSerial(CUBECELL_RX_PIN, -1);  // RX only

// CRSF to FC
AlfredoCRSF crsf;
crsf_channels_t crsfChannels = { 0 };

// Protocol constants
#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0xBB
#define MAX_PACKET_SIZE 32

// Packet parsing state
enum ParseState { WAIT_SYNC1, WAIT_SYNC2, WAIT_LEN, READ_DATA, WAIT_CRC };
ParseState parseState = WAIT_SYNC1;
uint8_t packetBuffer[MAX_PACKET_SIZE];
uint8_t packetLen = 0;
uint8_t packetIdx = 0;

// Data from CubeCell
int16_t rxForward = 0;      // -100 to 100
int16_t rxBackward = 0;     // -100 to 100
int16_t rxLeft = 0;         // -100 to 100
int16_t rxRight = 0;        // -100 to 100
int16_t rxSpeed = 0;        // 0 to 100
int16_t rxTemperature = 0;  // degrees * 10

uint32_t lastPacketTime = 0;
uint32_t packetCount = 0;
uint32_t lastCrsfTime = 0;

// Convert -100..100 to CRSF range
int16_t valueToCrsf(int16_t value) {
    return map(value, -100, 100, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
}

// Convert 0..100 to CRSF range
int16_t speedToCrsf(int16_t value) {
    return map(value, 0, 100, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX);
}

// Process received packet from CubeCell
void processPacket() {
    if (packetLen >= 12) {  // 6 values * 2 bytes each
        // Extract values (high byte first)
        rxForward     = (int16_t)((packetBuffer[0] << 8) | packetBuffer[1]);
        rxBackward    = (int16_t)((packetBuffer[2] << 8) | packetBuffer[3]);
        rxLeft        = (int16_t)((packetBuffer[4] << 8) | packetBuffer[5]);
        rxRight       = (int16_t)((packetBuffer[6] << 8) | packetBuffer[7]);
        rxSpeed       = (int16_t)((packetBuffer[8] << 8) | packetBuffer[9]);
        rxTemperature = (int16_t)((packetBuffer[10] << 8) | packetBuffer[11]);

        // Map to CRSF channels
        // Pitch: Forward/Backward combined
        int16_t pitch = rxForward - rxBackward;
        crsfChannels.ch1 = valueToCrsf(pitch);

        // Roll: Left/Right combined
        int16_t roll = rxRight - rxLeft;
        crsfChannels.ch0 = valueToCrsf(roll);

        // Throttle: Speed
        crsfChannels.ch2 = speedToCrsf(rxSpeed);

        // Yaw: stays centered
        crsfChannels.ch3 = CRSF_CHANNEL_VALUE_MID;

        lastPacketTime = millis();
        packetCount++;
    }
}

// Parse incoming byte from CubeCell
void parseCubecellByte(uint8_t b) {
    static uint8_t xorCrc = 0;

    switch (parseState) {
        case WAIT_SYNC1:
            if (b == SYNC_BYTE_1) {
                parseState = WAIT_SYNC2;
            }
            break;

        case WAIT_SYNC2:
            if (b == SYNC_BYTE_2) {
                parseState = WAIT_LEN;
            } else {
                parseState = WAIT_SYNC1;
            }
            break;

        case WAIT_LEN:
            if (b > 0 && b <= MAX_PACKET_SIZE) {
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
            if (packetIdx >= packetLen) {
                parseState = WAIT_CRC;
            }
            break;

        case WAIT_CRC:
            if (b == xorCrc) {
                processPacket();
            }
            parseState = WAIT_SYNC1;
            break;
    }
}

void sendChannels() {
    crsf.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &crsfChannels, sizeof(crsfChannels));
}

void setup() {
    // CRSF to FC at 420000 baud
    Serial.begin(420000);
    crsf.begin(Serial);

    // CubeCell input at 9600 baud
    cubecellSerial.begin(9600);

    delay(100);

    // Default safe values
    crsfChannels.ch0 = CRSF_CHANNEL_VALUE_MID;   // Roll
    crsfChannels.ch1 = CRSF_CHANNEL_VALUE_MID;   // Pitch
    crsfChannels.ch2 = CRSF_CHANNEL_VALUE_MIN;   // Throttle
    crsfChannels.ch3 = CRSF_CHANNEL_VALUE_MID;   // Yaw
    crsfChannels.ch4 = CRSF_CHANNEL_VALUE_MIN;   // Aux1
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
    // Read data from CubeCell
    while (cubecellSerial.available()) {
        uint8_t b = cubecellSerial.read();
        parseCubecellByte(b);
    }

    // Failsafe: if no data for 500ms, center sticks
    if ((millis() - lastPacketTime) > 500) {
        crsfChannels.ch0 = CRSF_CHANNEL_VALUE_MID;
        crsfChannels.ch1 = CRSF_CHANNEL_VALUE_MID;
        crsfChannels.ch2 = CRSF_CHANNEL_VALUE_MIN;
        crsfChannels.ch3 = CRSF_CHANNEL_VALUE_MID;
    }

    // Send CRSF frames at ~50Hz
    if ((millis() - lastCrsfTime) >= 20) {
        lastCrsfTime = millis();
        sendChannels();
    }

    yield();
}
