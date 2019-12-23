/**
 * Raspberry Pi Naza Decoder
 * 
 * Works with Raspbian and all Raspberry Pi models
 * 
 * GPS module is connected to "/dev/ttyS0" (pins 8, 10)
 */

#ifndef __RASPI_NAZA_DECODER_H__
#define __RASPI_NAZA_DECODER_H__

#include <iostream>
#include <unistd.h>
#include <string>
#include <math.h>
#include <fcntl.h>
#include <termios.h>

#define MESSAGE_HEADER_SIZE                 0x04
#define NAZA_MESSAGE_MAX_PAYLOAD_LENGTH     0x3A

using namespace std;

class NazaDecoder 
{
public:
    enum GPSPayloadPosition 
    {
        // date and time
        NAZA_MESSAGE_POS_DT = 0x04 - MESSAGE_HEADER_SIZE,

        // longitude (x10^7, degree decimal)
        NAZA_MESSAGE_POS_LO = 0x08 - MESSAGE_HEADER_SIZE,

        // latitude (x10^7, degree decimal)
        NAZA_MESSAGE_POS_LA = 0x0C - MESSAGE_HEADER_SIZE,

        // altitude (in millimeters)
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
        NAZA_MESSAGE_POS_PD = 0x2C - MESSAGE_HEADER_SIZE,

        // vertical DOP (see uBlox NAV-DOP message for details)
        NAZA_MESSAGE_POS_VD = 0x2E - MESSAGE_HEADER_SIZE,

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
        NAZA_MESSAGE_POS_XM = 0x3B - MESSAGE_HEADER_SIZE,

        // sequence number (not XORed), once there is a lock - increases with every message. When the lock is lost later LSB and MSB are swapped with every message.
        NAZA_MESSAGE_POS_SN = 0x3C - MESSAGE_HEADER_SIZE,

        // checksum, calculated the same way as for uBlox binary messages
        NAZA_MESSAGE_POS_CS = 0x3E - MESSAGE_HEADER_SIZE
    };

    enum MagnetometerPayloadPosition 
    {
        // magnetometer X axis data (signed)
        NAZA_MESSAGE_POS_CX = 0x04 - MESSAGE_HEADER_SIZE,

        // magnetometer Y axis data (signed)
        NAZA_MESSAGE_POS_CY = 0x06 - MESSAGE_HEADER_SIZE,

        // magnetometer Z axis data (signed)
        NAZA_MESSAGE_POS_CZ = 0x08 - MESSAGE_HEADER_SIZE
    };

    enum ModuleVersionPayloadPosition 
    {
        // firmware version
        NAZA_MESSAGE_POS_FW = 0x08 - MESSAGE_HEADER_SIZE,

        // hardware id
        NAZA_MESSAGE_POS_HW = 0x0C - MESSAGE_HEADER_SIZE
    };

    enum MessageType 
    {
        NAZA_MESSAGE_NONE_TYPE = 0x00,
        NAZA_MESSAGE_GPS_TYPE = 0x10,
        NAZA_MESSAGE_MAGNETOMETER_TYPE = 0x20,
        NAZA_MESSAGE_MODULE_VERSION_TYPE = 0x30
    };

    enum MessageSize 
    {
        NAZA_MESSAGE_GPS_SIZE = 0x3A,
        NAZA_MESSAGE_MAGNETOMETER_SIZE = 0x06,
        NAZA_MESSAGE_MODULE_VERSION_SIZE = 0x0C
    };

    enum FixType
    {
        NO_FIX = 0x00,
        FIX_2D = 0x02,
        FIX_3D = 0x03,
        FIX_DGPS = 0x04
    };

    struct VersionSchemeType 
    {
        uint8_t revision;
        uint8_t build;
        uint8_t minor;
        uint8_t major;
    };

    struct VersionType 
    {
        uint32_t version;
        VersionSchemeType scheme;
    };

    union GPSData 
    {


    };

    // Constructor (open serial port to communicate with GPS module)
    NazaDecoder()  : sequence(0), count(0), messageId(0), messageLength(0), checksum1(0), checksum2(0), magXMin(0), magXMax(0), magYMin(0), magYMax(0), longitude(0), latitude(0), altitude(0), speed(0), fix(NO_FIX), satellites(0), heading(0), courseOverGround(0), verticalSpeedIndicator(0), horizontalDilutionOfPrecision(
                0), verticalDilutionOfPrecision(0), year(0), month(0), day(0), hour(0), minute(0), second(0), lastLock(0), locked(0) 
    {        
	    serial_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	    if (serial_fd == -1)
	    {
		    //ERROR - CAN'T OPEN SERIAL PORT
		    printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	    }
	
	    //CONFIGURE THE UART
	    //The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	    //	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	    //	CSIZE:- CS5, CS6, CS7, CS8
	    //	CLOCAL - Ignore modem status lines
	    //	CREAD - Enable receiver
	    //	IGNPAR = Ignore characters with parity errors
	    //	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	    //	PARENB - Parity enable
	    //	PARODD - Odd parity (else even)
	    struct termios options;
	    tcgetattr(serial_fd, &options);
	    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	    options.c_iflag = IGNPAR;
	    options.c_oflag = 0;
	    options.c_lflag = 0;
	    tcflush(serial_fd, TCIFLUSH);
	    tcsetattr(serial_fd, TCSANOW, &options);
    }

    ~NazaDecoder()
    {
        //----- CLOSE THE UART -----
    	close(serial_fd);
    }

    // Read data from GPS module
    uint8_t Read()
    {
        //----- CHECK FOR ANY RX BYTES -----
	    if (serial_fd != -1)
	    {
		    // Read up to 1024 characters (1KB data) from the port
		    unsigned char rx_buffer[1024];
		    auto rx_length = read(serial_fd, (void*)rx_buffer, 1024);		//Filestream, buffer to store in, number of bytes to read (max)
		    if (rx_length < 0)
		    {
			    cout << "Cannot read from the port!" << endl;
		    }
    		else
	    	{
		    	//Bytes received
                for (int i = 0; i < rx_length; i++)
                {
                    auto decodedMessage = this->decode(rx_buffer[i]);
                    if (decodedMessage != NAZA_MESSAGE_NONE_TYPE)
                    {
                        return decodedMessage;
                    }                  
	    	    }
	        }
        }

        return NAZA_MESSAGE_NONE_TYPE;
    }

    /**
     * Gps API.
     */
    double getLatitude()                      { return this->latitude; }
    double getLongitude()                     { return this->longitude; }
    double getAltitude()                      { return this->altitude; }
    double getSpeed()                         { return this->speed; }
    FixType getFixType()                      { return this->fix; }
    uint8_t getSatellites()                   { return this->satellites; }
    double getCourseOverGround()              { return this->courseOverGround; }
    double getVerticalSpeedIndicator()        { return this->verticalSpeedIndicator; }
    double getHorizontalDilutionOfPrecision() { return this->horizontalDilutionOfPrecision; }
    double getVerticalDilutionOfPrecision()   { return this->verticalDilutionOfPrecision; }
    uint8_t getYear()                         { return this->year; }
    uint8_t getMonth()                        { return this->month; }
    uint8_t getDay()                          { return this->day; }

    // Note that for time between 16:00 and 23:59 the hour returned from GPS module is actually 00:00 - 7:59.
    uint8_t getHour()                         { return this->hour; }
    uint8_t getMinute()                       { return this->minute; }
    uint8_t getSecond()                       { return this->second; }

    /**
     * Magnetometer API
     */
    double getHeading()                       { return this->heading; }

    // Note that you need to read version numbers backwards (02 01 00 06 means v6.0.1.2)
    VersionType getFirmwareVersion()          { return this->firmwareVersion; }
    VersionType getHardwareVersion()          { return this->hardwareVersion; }

    uint8_t isLocked()                        { return this->locked; }
private:
    int16_t payload[NAZA_MESSAGE_MAX_PAYLOAD_LENGTH];
    int16_t sequence;
    int16_t count;
    int16_t messageId;
    int16_t messageLength;

    // checksum #1
    uint8_t checksum1;

    // checksum #2
    uint8_t checksum2;

    int16_t magXMin;
    int16_t magXMax;
    int16_t magYMin;
    int16_t magYMax;

    // longitude in degree decimal
    double longitude;

    // latitude in degree decimal
    double latitude;

    // altitude in m (from GPS)
    double altitude;

    // speed in m/s
    double speed;

    // fix type
    FixType fix;

    // number of satellites
    uint8_t satellites;

    // heading (not tilt compensated) in degrees
    double heading;

    // course over ground
    double courseOverGround;

    // vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
    double verticalSpeedIndicator;

    // horizontal dilution of precision
    double horizontalDilutionOfPrecision;

    // vertical dilution of precision
    double verticalDilutionOfPrecision;

    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    VersionType firmwareVersion;
    VersionType hardwareVersion;

    uint16_t lastLock;
    uint8_t locked;

	int serial_fd = -1;

    // Decode message from module
    uint8_t decode(int16_t input)
    {
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
        else if ((sequence == 3) && (((messageId == NAZA_MESSAGE_GPS_TYPE) && (input == NAZA_MESSAGE_GPS_SIZE)) || ((messageId == NAZA_MESSAGE_MAGNETOMETER_TYPE) && (input == NAZA_MESSAGE_MAGNETOMETER_SIZE)) || ((messageId == NAZA_MESSAGE_MODULE_VERSION_TYPE) && (input == NAZA_MESSAGE_MODULE_VERSION_SIZE)))) {
            messageLength = input;
            count = 0;
            updateChecksum(input);
            sequence++;
        } else if (sequence == 4) {
            payload[count++] = input;
            updateChecksum(input);
            if (count >= messageLength)
                sequence++;
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
                heading = atan2(y - ((magYMax + magYMin) / 2), x - ((magXMax + magXMin) / 2)); // ???? maybe bad function used insted of computeVectorAngle
            } else if (messageId == NAZA_MESSAGE_MODULE_VERSION_TYPE) {
                firmwareVersion.version = pack4(NAZA_MESSAGE_POS_FW, 0x00);
                hardwareVersion.version = pack4(NAZA_MESSAGE_POS_HW, 0x00);
            }
            return messageId;
        } else {
            return NAZA_MESSAGE_NONE_TYPE;
        }
    }

    int32_t pack4(uint8_t i, uint8_t mask)
    {
        struct {
            uint32_t d;
            uint8_t b[4];
        } v;
        
        for (int j = 0; j < 4; j++) {
            v.b[j] = payload[i + j] ^ mask;
        }

        return v.d;
    }

    int16_t pack2(uint8_t i, uint8_t mask)
    {
        struct {
            uint16_t d;
            uint8_t b[2];
        } v;
        
        for (int j = 0; j < 2; j++) {
            v.b[j] = payload[i + j] ^ mask;
        }

        return v.d;
    }

    void updateChecksum(int16_t input)
    {
        checksum1 += input;
        checksum2 += checksum1;
    }
};

#endif /* __RASPI_NAZA_DECODER_H__ */
