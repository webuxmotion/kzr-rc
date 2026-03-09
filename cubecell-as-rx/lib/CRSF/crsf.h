#ifndef CRSF_H
#define CRSF_H

#include <Arduino.h>

// CRSF Protocol Constants
#define CRSF_SYNC_BYTE          0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_LINK_STATISTICS    0x14

#define CRSF_PAYLOAD_SIZE_MAX   62
#define CRSF_FRAME_SIZE_MAX     64

#define CRSF_CHANNEL_VALUE_MIN  172
#define CRSF_CHANNEL_VALUE_MID  992
#define CRSF_CHANNEL_VALUE_MAX  1811

#define CRSF_NUM_CHANNELS       16

// RC Channels packed frame is 22 bytes payload + 2 header + 1 crc = 25 bytes total
#define CRSF_RC_CHANNELS_PACKED_SIZE 22

class CRSF {
public:
    CRSF();

    // Initialize CRSF with serial port (uses hardware UART pins)
    void begin(HardwareSerial* serial, uint32_t baudRate = 420000);

    // Set channel value (channel 0-15, value 172-1811)
    void setChannel(uint8_t channel, uint16_t value);

    // Set channel value from microseconds (1000-2000us range)
    void setChannelUs(uint8_t channel, uint16_t us);

    // Get current channel value
    uint16_t getChannel(uint8_t channel);

    // Send RC channels frame to FC
    void sendChannels();

    // Set all channels to center (mid position)
    void centerAllChannels();

    // Convert microseconds (1000-2000) to CRSF value (172-1811)
    static uint16_t usToChannel(uint16_t us);

    // Convert CRSF value to microseconds
    static uint16_t channelToUs(uint16_t value);

private:
    HardwareSerial* _serial;
    uint16_t _channels[CRSF_NUM_CHANNELS];
    uint8_t _frameBuffer[CRSF_FRAME_SIZE_MAX];

    // CRC8 calculation with polynomial 0xD5
    uint8_t crc8(const uint8_t* data, uint8_t len);

    // Pack 16 channels (11-bit each) into 22 bytes
    void packChannels(uint8_t* payload);
};

#endif // CRSF_H
