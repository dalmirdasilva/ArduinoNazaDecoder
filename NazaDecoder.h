/**
 * Arduino Naza Decoder
 *
 * Inspired by the Pawelsky's work.
 */

#ifndef __ARDUINO_NAZA_DECODER_H__
#define __ARDUINO_NAZA_DECODER_H__

#include <Arduino.h>

#define MESSAGE_HEADER_SIZE 0x04

class NazaDecoder {

public:

    typedef enum {

        // date and time
        NAZA_MESSAGE_POS_DT = 0x04 - MESSAGE_HEADER_SIZE,

        // longitude (x10^7, degree decimal)
        NAZA_MESSAGE_POS_LO = 0x08 - MESSAGE_HEADER_SIZE,

        // latitude (x10^7, degree decimal)
        NAZA_MESSAGE_POS_LA = 0x0c - MESSAGE_HEADER_SIZE,

        // altitude (in milimeters)
        NAZA_MESSAGE_POS_AL = 0x10 - MESSAGE_HEADER_SIZE,

        // horizontal accuracy estimate (see uBlox NAV-POSLLH message for details)
        NAZA_MESSAGE_POS_HA = 0x14 - MESSAGE_HEADER_SIZE,

        // vertical accuracy estimate (see uBlox NAV-POSLLH message for details)
        NAZA_MESSAGE_POS_VA = 0x18 - MESSAGE_HEADER_SIZE,

        // NED north velocity (see uBlox NAV-VELNED message for details)
        NAZA_MESSAGE_POS_NV = 0x20 - MESSAGE_HEADER_SIZE,

        // NED east velocity (see uBlox NAV-VELNED message for details)
        NAZA_MESSAGE_POS_EV = 0x24 - MESSAGE_HEADER_SIZE,

        // NED down velocity (see uBlox NAV-VELNED message for details)
        NAZA_MESSAGE_POS_DV = 0x28 - MESSAGE_HEADER_SIZE,

        // position DOP (see uBlox NAV-DOP message for details)
        NAZA_MESSAGE_POS_PD = 0x2c - MESSAGE_HEADER_SIZE,

        // vertical DOP (see uBlox NAV-DOP message for details)
        NAZA_MESSAGE_POS_VD = 0x2e - MESSAGE_HEADER_SIZE,

        // northing DOP (see uBlox NAV-DOP message for details)
        NAZA_MESSAGE_POS_ND = 0x30 - MESSAGE_HEADER_SIZE,

        //easting DOP (see uBlox NAV-DOP message for details)
        NAZA_MESSAGE_POS_ED = 0x32 - MESSAGE_HEADER_SIZE,

        // number of satellites (not XORed)
        NAZA_MESSAGE_POS_NS = 0x34 - MESSAGE_HEADER_SIZE,

        // fix type (0 - no lock, 2 - 2D lock, 3 - 3D lock, not sure if other values can be expected - see uBlox NAV-SOL message for details)
        NAZA_MESSAGE_POS_FT = 0x36 - MESSAGE_HEADER_SIZE,

        // fix status flags (see uBlox NAV-SOL message for details)
        NAZA_MESSAGE_POS_SF = 0x38 - MESSAGE_HEADER_SIZE,

        // XOR mask
        NAZA_MESSAGE_POS_XM = 0x3b - MESSAGE_HEADER_SIZE,

        // sequence number (not XORed), once there is a lock - increases with every message. When the lock is lost later LSB and MSB are swapped with every message.
        NAZA_MESSAGE_POS_SN = 0x3c - MESSAGE_HEADER_SIZE,

        // checksum, calculated the same way as for uBlox binary messages
        NAZA_MESSAGE_POS_CS = 0x3e - MESSAGE_HEADER_SIZE
    } GPSPayloadPosition;

    typedef enum {

        // compass X axis data (signed)
        NAZA_MESSAGE_POS_CX = 0x04 - MESSAGE_HEADER_SIZE,

        // compass Y axis data (signed)
        NAZA_MESSAGE_POS_CY = 0x06 - MESSAGE_HEADER_SIZE,

        // compass Z axis data (signed)
        NAZA_MESSAGE_POS_CZ = 0x08 - MESSAGE_HEADER_SIZE
    } CompassPayloadPosition;

    typedef enum {

        // firmware version
        NAZA_MESSAGE_POS_FW = 0x08 - MESSAGE_HEADER_SIZE,

        // hardware id
        NAZA_MESSAGE_POS_HW = 0x0c - MESSAGE_HEADER_SIZE
    } ModuleVersionPayloadPosition;

    typedef enum {
        NAZA_MESSAGE_NONE_TYPE = 0x00,
        NAZA_MESSAGE_GPS_TYPE = 0x10,
        NAZA_MESSAGE_COMPASS_TYPE = 0x20,
        NAZA_MESSAGE_MODULE_VERSION_TYPE = 0x30
    } MessageType;

    typedef enum {
        NAZA_MESSAGE_GPS_SIZE = 0x3a,
        NAZA_MESSAGE_COMPASS_SIZE = 0x06,
        NAZA_MESSAGE_MODULE_VERSION_SIZE = 0x0c
    } MessageSize;

    typedef enum {
        NO_FIX = 0,
        FIX_2D = 2,
        FIX_3D = 3,
        FIX_DGPS = 4
    } FixType;

    typedef struct {
        uint8_t major;
        uint8_t minor;
        uint8_t build;
        uint8_t revision;
    } VersionSchemeType;

    typedef union {
        uint32_t version;
        VersionSchemeType scheme;
    } VersionType;

    NazaDecoder();

    uint8_t decode(int16_t input);
    double getLat();
    double getLon();
    double getGpsAlt();
    double getSpeed();
    FixType getFixType();
    uint8_t getNumSat();
    double getHeading();
    double getCog();
    double getGpsVsi();
    double getHdop();
    double getVdop();
    uint8_t getYear();
    uint8_t getMonth();
    uint8_t getDay();

    // Note that for time between 16:00 and 23:59 the hour returned from GPS module is actually 00:00 - 7:59.
    uint8_t getHour();
    uint8_t getMinute();
    uint8_t getSecond();

    // Note that you need to read version numbers backwards (02 01 00 06 means v6.0.1.2)
    VersionType getFirmwareVersion();
    VersionType getHardwareVersion();

private:
    int16_t payload[58];
    int16_t seq;
    int16_t cnt;
    int16_t msgId;
    int16_t msgLen;

    // checksum #1
    uint8_t cs1;

    // checksum #2
    uint8_t cs2;
    int16_t magXMin;
    int16_t magXMax;
    int16_t magYMin;
    int16_t magYMax;

    // longitude in degree decimal
    double lon;

    // latitude in degree decimal
    double lat;

    // altitude in m (from GPS)
    double gpsAlt;

    // speed in m/s
    double spd;

    // fix type
    FixType fix;

    // number of satellites
    uint8_t sat;

    // heading (not tilt compensated) in degrees
    double heading;

    // course over ground
    double cog;

    // vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
    double gpsVsi;

    // horizontal dilution of precision
    double hdop;

    // vertical dilution of precision
    double vdop;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    VersionType firmwareVersion;
    VersionType hardwareVersion;

    int32_t pack4(uint8_t i, uint8_t mask);

    int16_t pack2(uint8_t i, uint8_t mask);

    void updateChecksum(int16_t input);
};

#endif /* __ARDUINO_NAZA_DECODER_H__ */
