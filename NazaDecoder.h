/**
 * Arduino Naza Decoder
 *
 * Inspired in the Pawelsky's work.
 */

#ifndef __ARDUINO_NAZA_DECODER_H__
#define __ARDUINO_NAZA_DECODER_H__

#include <Arduino.h>

class NazaDecoder {

public:

    typedef enum {
        NAZA_MESSAGE_NONE = 0x00,
        NAZA_MESSAGE_GPS = 0x10,
        NAZA_MESSAGE_COMPASS = 0x20
    } MessageType;

    typedef enum {
        NO_FIX = 0,
        FIX_2D = 2,
        FIX_3D = 3,
        FIX_DGPS = 4
    } fixType;

    NazaDecoder();

    uint8_t decode(int16_t input);
    double getLat();
    double getLon();
    double getGpsAlt();
    double getSpeed();
    fixType getFixType();
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
    fixType fix;

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

    int32_t pack4(uint8_t i, uint8_t mask);

    int16_t pack2(uint8_t i, uint8_t mask);

    void updateChecksum(int16_t input);
};

#endif /* __ARDUINO_NAZA_DECODER_H__ */
