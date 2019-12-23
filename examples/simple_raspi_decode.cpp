#include "NazaDecoder.hpp"

#include <iostream>
#include <thread>
#include <chrono>

int main()
{
	NazaDecoder myGPS;
	
	while (1)
	{
		uint8_t decodedMessage = myGPS.Read();
		switch (decodedMessage) 
		{
			case NazaDecoder::NAZA_MESSAGE_GPS_TYPE:
			//if (myGPS.isLocked())
			{
				std::cout << "Latitude: " << myGPS.getLatitude() << endl;
				std::cout << "Longitude: " << myGPS.getLongitude() << endl;
				std::cout << "Altitude: " << myGPS.getAltitude() << endl;
				std::cout << "Speed: " << myGPS.getSpeed() << endl;
				std::cout << "FixType: " << myGPS.getFixType() << endl;
				std::cout << "Satellites: " << (int)myGPS.getSatellites() << std::endl;
				std::cout << (int)myGPS.getDay() << "-" << (int)myGPS.getMonth() << "-" << (int)myGPS.getYear() << ", " << (int)myGPS.getHour() << ":" << (int)myGPS.getMinute() << ":" << (int)myGPS.getSecond() << std::endl;
				std::cout << "Locked: " << (int)myGPS.isLocked() << std::endl;
				std::cout << std::endl;
			}
				break;
			case NazaDecoder::NAZA_MESSAGE_MAGNETOMETER_TYPE:
						//if (myGPS.isLocked())
			//{

				std::cout << "Heading: " << myGPS.getHeading() << std::endl;
				std::cout << std::endl;
			//}
				break;
			/*case NazaDecoder::NAZA_MESSAGE_MODULE_VERSION_TYPE:
			if (myGPS.isLocked())
			{
				NazaDecoder::VersionSchemeType fs = myGPS.getFirmwareVersion().scheme;
				NazaDecoder::VersionSchemeType hs = myGPS.getHardwareVersion().scheme;
				std::cout << "Firmware version: v" << fs.major << "." << fs.minor << "." << fs.build << "." << fs.revision << endl;
				std::cout << "Hardware version: v" << hs.major << "." << hs.minor << "." << hs.build << "." << hs.revision << endl;
				std::cout << std::endl;
			}
				break;*/
		}
	}
	
	return 0;
}
