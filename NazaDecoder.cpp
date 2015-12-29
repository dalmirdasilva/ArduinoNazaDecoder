#include <Arduino.h>
#include "NazaDecoder.h"

NazaDecoder::NazaDecoder()
        : seq(0), cnt(0), msgId(0), msgLen(0), cs1(0), cs2(0), magXMin(0), magXMax(0), magYMin(0), magYMax(0), lon(0), lat(0), gpsAlt(0), spd(0), fix(NO_FIX), sat(0), heading(0), cog(0), gpsVsi(0), hdop(0), vdop(0), year(0), month(0), day(0), hour(0), minute(0), second(0) {
}
void NazaDecoder::updateChecksum(int16_t input) {
    cs1 += input;
    cs2 += cs1;
}

double NazaDecoder::getLat() {
    return lat;
}

double NazaDecoder::getLon() {
    return lon;
}

double NazaDecoder::getGpsAlt() {
    return gpsAlt;
}

double NazaDecoder::getSpeed() {
    return spd;
}

NazaDecoder::fixType NazaDecoder::getFixType() {
    return fix;
}

uint8_t NazaDecoder::getNumSat() {
    return sat;
}

double NazaDecoder::getHeading() {
    return heading;
}

double NazaDecoder::getCog() {
    return cog;
}

double NazaDecoder::getGpsVsi() {
    return gpsVsi;
}

double NazaDecoder::getHdop() {
    return hdop;
}

double NazaDecoder::getVdop() {
    return vdop;
}

uint8_t NazaDecoder::getYear() {
    return year;
}

uint8_t NazaDecoder::getMonth() {
    return month;
}

uint8_t NazaDecoder::getDay() {
    return day;
}

uint8_t NazaDecoder::getHour() {
    return hour;
}

uint8_t NazaDecoder::getMinute() {
    return minute;
}

uint8_t NazaDecoder::getSecond() {
    return second;
}

uint8_t NazaDecoder::decode(int16_t input) {

    // header (part 1 - 0x55)
    if ((seq == 0) && (input == 0x55)) {
        seq++;
    }

    // header (part 2 - 0xAA)
    else if ((seq == 1) && (input == 0xAA)) {
        cs1 = 0;
        cs2 = 0;
        seq++;
    } else if (seq == 2) {
        msgId = input;
        updateChecksum(input);
        seq++;
    }

    // message id
    // message payload length (should match message id)
    // store payload in buffer
    else if ((seq == 3) && (((msgId == 0x10) && (input == 0x3A)) || ((msgId == 0x20) && (input == 0x06)))) {
        msgLen = input;
        cnt = 0;
        updateChecksum(input);
        seq++;
    } else if (seq == 4) {
        payload[cnt++] = input;
        updateChecksum(input);
        if (cnt >= msgLen) {
            seq++;
        }
    }

    // verify checksum #1
    else if ((seq == 5) && (input == cs1)) {
        seq++;
    }

    // verify checksum #2
    else if ((seq == 6) && (input == cs2)) {
        seq++;
    } else {
        seq = 0;
    }

    // all data in buffer
    if (seq == 7) {
        seq = 0;

        // Decode GPS data
        if (msgId == NAZA_MESSAGE_GPS) {
            uint8_t mask = payload[55];
            uint32_t time = pack4(0, mask);
            second = time & 0x3f;
            time >>= 6;
            minute = time & 0x3f;
            time >>= 6;
            hour = time & 0x0f;
            time >>= 4;
            day = time & 0x1f;
            time >>= 5;
            if (hour > 7) {
                day++;
            }
            month = time & 0x0f;
            time >>= 4;
            year = time & 0x7f;
            lon = (double) pack4(4, mask) / 10000000;
            lat = (double) pack4(8, mask) / 10000000;
            gpsAlt = (double) pack4(12, mask) / 1000;
            double nVel = (double) pack4(28, mask) / 100;
            double eVel = (double) pack4(32, mask) / 100;
            spd = sqrt(nVel * nVel + eVel * eVel);
            cog = atan2(eVel, nVel) * 180.0 / M_PI;
            if (cog < 0) {
                cog += 360.0;
            }
            gpsVsi = -(double) pack4(36, mask) / 100;
            vdop = (double) pack2(42, mask) / 100;
            double ndop = (double) pack2(44, mask) / 100;
            double edop = (double) pack2(46, mask) / 100;
            hdop = sqrt(ndop * ndop + edop * edop);
            sat = payload[48];
            uint8_t fixType = payload[50] ^ mask;
            uint8_t fixFlags = payload[52] ^ mask;
            switch (fixType) {
            case 2:
                fix = FIX_2D;
                break;
            case 3:
                fix = FIX_3D;
                break;
            default:
                fix = NO_FIX;
                break;
            }
            if ((fix != NO_FIX) && (fixFlags & 0x02))
                fix = FIX_DGPS;
        }

        // Decode compass data (not tilt compensated)
        else if (msgId == NAZA_MESSAGE_COMPASS) {
            uint8_t mask = payload[4];
            mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7));
            int16_t x = pack2(0, mask);
            int16_t y = pack2(2, mask);
            if (x > magXMax)
                magXMax = x;
            if (x < magXMin)
                magXMin = x;
            if (y > magYMax)
                magYMax = y;
            if (y < magYMin)
                magYMin = y;
            heading = -atan2(y - ((magYMax + magYMin) / 2), x - ((magXMax + magXMin) / 2)) * 180.0 / M_PI;
            if (heading < 0) {
                heading += 360.0;
            }
        }
        return msgId;
    } else {
        return NAZA_MESSAGE_NONE;
    }
}

int32_t NazaDecoder::pack4(uint8_t i, uint8_t mask) {
    union {
        uint32_t d;
        uint8_t b[4];
    } v;
    for (int j = 0; j < 4; j++)
        v.b[j] = payload[i + j] ^ mask;
    return v.d;
}

int16_t NazaDecoder::pack2(uint8_t i, uint8_t mask) {
    union {
        uint16_t d;
        uint8_t b[2];
    } v;
    for (int j = 0; j < 2; j++) {
        v.b[j] = payload[i + j] ^ mask;
    }
    return v.d;
}

