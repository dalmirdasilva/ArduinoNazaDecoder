#include <Arduino.h>
#include "NazaDecoder.h"

NazaDecoder::NazaDecoder()
        : sequence(0), count(0), messageId(0), messageLength(0), checksum1(0), checksum2(0), magXMin(0), magXMax(0), magYMin(0), magYMax(0), longitude(0), latitude(0), altitude(0), speed(0), fix(NO_FIX), satellites(0), heading(0), courseOverGround(0), verticalSpeedIndicator(0), horizontalDilutionOfPrecision(
                0), verticalDilutionOfPrecision(0), year(0), month(0), day(0), hour(0), minute(0), second(0), lastLock(0), locked(0) {
}

double NazaDecoder::getLatitude() {
    return latitude;
}

double NazaDecoder::getLongitude() {
    return longitude;
}

double NazaDecoder::getAltitude() {
    return altitude;
}

double NazaDecoder::getSpeed() {
    return speed;
}

NazaDecoder::FixType NazaDecoder::getFixType() {
    return fix;
}

uint8_t NazaDecoder::getSatellites() {
    return satellites;
}

double NazaDecoder::getHeading() {
    return heading;
}

double NazaDecoder::getCourseOverGround() {
    return courseOverGround;
}

double NazaDecoder::getVerticalSpeedIndicator() {
    return verticalSpeedIndicator;
}

double NazaDecoder::getHorizontalDilutionOfPrecision() {
    return horizontalDilutionOfPrecision;
}

double NazaDecoder::getVerticalDilutionOfPrecision() {
    return verticalDilutionOfPrecision;
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

NazaDecoder::VersionType NazaDecoder::getFirmwareVersion() {
    return firmwareVersion;
}

NazaDecoder::VersionType NazaDecoder::getHardwareVersion() {
    return hardwareVersion;
}

uint8_t NazaDecoder::isLocked() {
    return locked;
}

uint8_t NazaDecoder::decode(int16_t input) {

    // header (part 1 - 0x55)
    if ((sequence == 0) && (input == 0x55)) {
        sequence++;
    }

    // header (part 2 - 0xaa)
    else if ((sequence == 1) && (input == 0xaa)) {
        checksum1 = 0;
        checksum2 = 0;
        sequence++;
    } else if (sequence == 2) {
        messageId = input;
        updateChecksum(input);
        sequence++;
    }

    // message id
    // message payload length (should match message id)
    // store payload in buffer
    else if ((sequence == 3)
            && (((messageId == NAZA_MESSAGE_GPS_TYPE) && (input == NAZA_MESSAGE_GPS_SIZE)) || ((messageId == NAZA_MESSAGE_MAGNETOMETER_TYPE) && (input == NAZA_MESSAGE_MAGNETOMETER_SIZE)) || ((messageId == NAZA_MESSAGE_MODULE_VERSION_TYPE) && (input == NAZA_MESSAGE_MODULE_VERSION_SIZE)))) {
        messageLength = input;
        count = 0;
        updateChecksum(input);
        sequence++;
    } else if (sequence == 4) {
        payload[count++] = input;
        updateChecksum(input);
        if (count >= messageLength) {
            sequence++;
        }
    }

    // verify checksum #1
    else if ((sequence == 5) && (input == checksum1)) {
        sequence++;
    }

    // verify checksum #2
    else if ((sequence == 6) && (input == checksum2)) {
        sequence++;
    } else {
        sequence = 0;
    }

    // all data in buffer
    if (sequence == 7) {
        sequence = 0;

        // Decode GPS data
        if (messageId == NAZA_MESSAGE_GPS_TYPE) {
            uint8_t mask = payload[NAZA_MESSAGE_POS_XM];
            uint32_t time = pack4(NAZA_MESSAGE_POS_DT, mask);
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
            longitude = (double) pack4(NAZA_MESSAGE_POS_LO, mask) / 10000000;
            latitude = (double) pack4(NAZA_MESSAGE_POS_LA, mask) / 10000000;
            altitude = (double) pack4(NAZA_MESSAGE_POS_AL, mask) / 1000;
            double northVelocity = (double) pack4(NAZA_MESSAGE_POS_NV, mask) / 100;
            double eastVelocity = (double) pack4(NAZA_MESSAGE_POS_EV, mask) / 100;
            speed = sqrt(northVelocity * northVelocity + eastVelocity * eastVelocity);
            courseOverGround = atan2(eastVelocity, northVelocity) * 180.0 / M_PI;
            if (courseOverGround < 0) {
                courseOverGround += 360.0;
            }
            verticalSpeedIndicator = -(double) pack4(NAZA_MESSAGE_POS_DV, mask) / 100;
            verticalDilutionOfPrecision = (double) pack2(NAZA_MESSAGE_POS_VD, mask) / 100;
            double ndop = (double) pack2(NAZA_MESSAGE_POS_ND, mask) / 100;
            double edop = (double) pack2(NAZA_MESSAGE_POS_ED, mask) / 100;
            horizontalDilutionOfPrecision = sqrt(ndop * ndop + edop * edop);
            satellites = payload[NAZA_MESSAGE_POS_NS];
            uint8_t fixType = payload[NAZA_MESSAGE_POS_FT] ^ mask;
            uint8_t fixFlags = payload[NAZA_MESSAGE_POS_SF] ^ mask;
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
            if ((fix != NO_FIX) && (fixFlags & 0x02)) {
                fix = FIX_DGPS;
            }
            uint16_t lock = pack2(NAZA_MESSAGE_POS_SN, 0x00);
            locked = (lock == lastLock + 1);
            lastLock = lock;
        }

        // Decode magnetometer data (not tilt compensated)
        // To calculate the heading (not tilt compensated) you need to do atan2 on the resulting y any a values, convert radians to degrees and add 360 if the result is negative.
        else if (messageId == NAZA_MESSAGE_MAGNETOMETER_TYPE) {
            uint8_t mask = payload[4];
            mask = (((mask ^ (mask >> 4)) & 0x0F) | ((mask << 3) & 0xF0)) ^ (((mask & 0x01) << 3) | ((mask & 0x01) << 7));
            int16_t x = pack2(NAZA_MESSAGE_POS_CX, mask);
            int16_t y = pack2(NAZA_MESSAGE_POS_CY, mask);
            if (x > magXMax) {
                magXMax = x;
            }
            if (x < magXMin) {
                magXMin = x;
            }
            if (y > magYMax) {
                magYMax = y;
            }
            if (y < magYMin) {
                magYMin = y;
            }
            heading = -atan2(y - ((magYMax + magYMin) / 2), x - ((magXMax + magXMin) / 2)) * 180.0 / M_PI;
            if(heading < 0) heading += 360.0; 
        } else if (messageId == NAZA_MESSAGE_MODULE_VERSION_TYPE) {
            firmwareVersion.version = pack4(NAZA_MESSAGE_POS_FW, 0x00);
            hardwareVersion.version = pack4(NAZA_MESSAGE_POS_HW, 0x00);
        }
        return messageId;
    } else {
        return NAZA_MESSAGE_NONE_TYPE;
    }
}

void NazaDecoder::updateChecksum(int16_t input) {
    checksum1 += input;
    checksum2 += checksum1;
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

