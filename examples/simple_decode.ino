#include <NazaDecoder.h>

NazaDecoder naza = NazaDecoder();

void setup() {
  Serial.begin(115200);
}

void loop() {
  if(Serial.available()) {
    uint8_t decodedMessage = naza.decode(Serial.read());
    switch (decodedMessage) {
      case NazaDecoder::NAZA_MESSAGE_GPS_TYPE:
        Serial.print("Latitude: ");
        Serial.print(naza.getLatitude(), 7);
        Serial.print(", Longitude: ");
        Serial.print(naza.getLongitude(), 7);
        Serial.print(", Altitude: ");
        Serial.print(naza.getAltitude(), 7);
        Serial.print(", FixType: ");
        Serial.print(naza.getFixType());
        Serial.print(", Satellites: ");
        Serial.println(naza.getSatellites());
        Serial.print(", Locked: ");
        Serial.println(naza.isLocked());
        break;
      case NazaDecoder::NAZA_MESSAGE_MAGNETOMETER_TYPE:
        Serial.print("Heading: ");
        Serial.println(naza.getHeading(), 2);
        break;
      case NazaDecoder::NAZA_MESSAGE_MODULE_VERSION_TYPE:
        NazaDecoder::VersionSchemeType fs = naza.getFirmwareVersion().scheme;
        NazaDecoder::VersionSchemeType hs = naza.getHardwareVersion().scheme;
        Serial.print("Firmware version: v");
        Serial.print(fs.major);
        Serial.print(".");
        Serial.print(fs.minor);
        Serial.print(".");
        Serial.print(fs.build);
        Serial.print(".");
        Serial.println(fs.revision);
        Serial.print("Hardware version: v");
        Serial.print(hs.major);
        Serial.print(".");
        Serial.print(hs.minor);
        Serial.print(".");
        Serial.print(hs.build);
        Serial.print(".");
        Serial.println(hs.revision);
        break;
    }
  }
}
