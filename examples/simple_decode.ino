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
        Serial.print("Lat: ");
        Serial.print(naza.getLat(), 7);
        Serial.print(", Lon: ");
        Serial.print(naza.getLon(), 7);
        Serial.print(", Alt: ");
        Serial.print(naza.getGpsAlt(), 7);
        Serial.print(", Fix: ");
        Serial.print(naza.getFixType());
        Serial.print(", Sat: ");
        Serial.println(naza.getNumSat());
        Serial.print(", Locked: ");
        Serial.println(naza.isLocked());
        break;
      case NazaDecoder::NAZA_MESSAGE_COMPASS_TYPE:
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
