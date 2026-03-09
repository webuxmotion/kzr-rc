#include "crsf.h"

// CRC8 lookup table with polynomial 0xD5
static const uint8_t crc8_lut[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

CRSF::CRSF() : _serial(nullptr) {
    centerAllChannels();
}

void CRSF::begin(HardwareSerial* serial, uint32_t baudRate) {
    _serial = serial;
    _serial->begin(baudRate);
}

void CRSF::setChannel(uint8_t channel, uint16_t value) {
    if (channel < CRSF_NUM_CHANNELS) {
        // Constrain value to valid CRSF range
        if (value < CRSF_CHANNEL_VALUE_MIN) value = CRSF_CHANNEL_VALUE_MIN;
        if (value > CRSF_CHANNEL_VALUE_MAX) value = CRSF_CHANNEL_VALUE_MAX;
        _channels[channel] = value;
    }
}

void CRSF::setChannelUs(uint8_t channel, uint16_t us) {
    setChannel(channel, usToChannel(us));
}

uint16_t CRSF::getChannel(uint8_t channel) {
    if (channel < CRSF_NUM_CHANNELS) {
        return _channels[channel];
    }
    return CRSF_CHANNEL_VALUE_MID;
}

void CRSF::centerAllChannels() {
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++) {
        _channels[i] = CRSF_CHANNEL_VALUE_MID;
    }
}

uint16_t CRSF::usToChannel(uint16_t us) {
    // Convert 1000-2000us to 172-1811 CRSF range
    // Formula: crsf = (us - 1000) * 1639 / 1000 + 172
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    return ((uint32_t)(us - 1000) * 1639 / 1000) + CRSF_CHANNEL_VALUE_MIN;
}

uint16_t CRSF::channelToUs(uint16_t value) {
    // Convert 172-1811 CRSF range to 1000-2000us
    if (value < CRSF_CHANNEL_VALUE_MIN) value = CRSF_CHANNEL_VALUE_MIN;
    if (value > CRSF_CHANNEL_VALUE_MAX) value = CRSF_CHANNEL_VALUE_MAX;
    return ((uint32_t)(value - CRSF_CHANNEL_VALUE_MIN) * 1000 / 1639) + 1000;
}

uint8_t CRSF::crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8_lut[crc ^ data[i]];
    }
    return crc;
}

void CRSF::packChannels(uint8_t* payload) {
    // Pack 16 x 11-bit channels into 22 bytes
    // Each channel is 11 bits, total = 176 bits = 22 bytes
    payload[0]  = (uint8_t)((_channels[0] & 0x07FF));
    payload[1]  = (uint8_t)((_channels[0] & 0x07FF) >> 8  | (_channels[1] & 0x07FF) << 3);
    payload[2]  = (uint8_t)((_channels[1] & 0x07FF) >> 5  | (_channels[2] & 0x07FF) << 6);
    payload[3]  = (uint8_t)((_channels[2] & 0x07FF) >> 2);
    payload[4]  = (uint8_t)((_channels[2] & 0x07FF) >> 10 | (_channels[3] & 0x07FF) << 1);
    payload[5]  = (uint8_t)((_channels[3] & 0x07FF) >> 7  | (_channels[4] & 0x07FF) << 4);
    payload[6]  = (uint8_t)((_channels[4] & 0x07FF) >> 4  | (_channels[5] & 0x07FF) << 7);
    payload[7]  = (uint8_t)((_channels[5] & 0x07FF) >> 1);
    payload[8]  = (uint8_t)((_channels[5] & 0x07FF) >> 9  | (_channels[6] & 0x07FF) << 2);
    payload[9]  = (uint8_t)((_channels[6] & 0x07FF) >> 6  | (_channels[7] & 0x07FF) << 5);
    payload[10] = (uint8_t)((_channels[7] & 0x07FF) >> 3);
    payload[11] = (uint8_t)((_channels[8] & 0x07FF));
    payload[12] = (uint8_t)((_channels[8] & 0x07FF) >> 8  | (_channels[9] & 0x07FF) << 3);
    payload[13] = (uint8_t)((_channels[9] & 0x07FF) >> 5  | (_channels[10] & 0x07FF) << 6);
    payload[14] = (uint8_t)((_channels[10] & 0x07FF) >> 2);
    payload[15] = (uint8_t)((_channels[10] & 0x07FF) >> 10 | (_channels[11] & 0x07FF) << 1);
    payload[16] = (uint8_t)((_channels[11] & 0x07FF) >> 7  | (_channels[12] & 0x07FF) << 4);
    payload[17] = (uint8_t)((_channels[12] & 0x07FF) >> 4  | (_channels[13] & 0x07FF) << 7);
    payload[18] = (uint8_t)((_channels[13] & 0x07FF) >> 1);
    payload[19] = (uint8_t)((_channels[13] & 0x07FF) >> 9  | (_channels[14] & 0x07FF) << 2);
    payload[20] = (uint8_t)((_channels[14] & 0x07FF) >> 6  | (_channels[15] & 0x07FF) << 5);
    payload[21] = (uint8_t)((_channels[15] & 0x07FF) >> 3);
}

void CRSF::sendChannels() {
    if (_serial == nullptr) return;

    // Build CRSF frame
    // [sync][len][type][payload...][crc]
    // len = type + payload + crc = 1 + 22 + 1 = 24

    _frameBuffer[0] = CRSF_SYNC_BYTE;
    _frameBuffer[1] = 24; // length: type(1) + payload(22) + crc(1)
    _frameBuffer[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    // Pack channels into payload (bytes 3-24)
    packChannels(&_frameBuffer[3]);

    // Calculate CRC over type + payload (bytes 2-24)
    _frameBuffer[25] = crc8(&_frameBuffer[2], 23);

    // Send frame (26 bytes total)
    _serial->write(_frameBuffer, 26);
}
