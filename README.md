# NazaDecoder


## Objective

Decode DJI NAZA GPS and MAGNETOMETER messages with all models Raspberry Pi or Arduino.

## How to install and run example?

Steps (on the console):

```bash
$ git clone git@github.com:markub3327/DJINazaGPSDecoder.git
$ cd DJINazaGPSDecoder
$ g++ main.cpp -o GPS_Example -lm
$ ./GPS_Example
```

## Examples

```cpp
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
				std::cout << "Latitude: " << myGPS.getLatitude() << endl;
				std::cout << "Longitude: " << myGPS.getLongitude() << endl;
				std::cout << "Altitude: " << myGPS.getAltitude() << endl;
				std::cout << "Speed: " << myGPS.getSpeed() << endl;
				std::cout << "FixType: " << myGPS.getFixType() << endl;
				std::cout << "Satellites: " << (int)myGPS.getSatellites() << std::endl;
				std::cout << (int)myGPS.getDay() << "-" << (int)myGPS.getMonth() << "-" << (int)myGPS.getYear() << ", " << (int)myGPS.getHour() << ":" << (int)myGPS.getMinute() << ":" << (int)myGPS.getSecond() << std::endl;
				std::cout << "Locked: " << (int)myGPS.isLocked() << std::endl;
				std::cout << std::endl;
				break;
			case NazaDecoder::NAZA_MESSAGE_MAGNETOMETER_TYPE:
				std::cout << "Heading: " << myGPS.getHeading() << std::endl;
				std::cout << std::endl;
				break;
		}
	}
	
	return 0;
}

```
