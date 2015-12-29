#include <NazaDecoder.h>

NazaDecoder naza = NazaDecoder();

void setup() {
  Serial.begin(115200);
}

void loop() {
  if(Serial.available()) {
    uint8_t decodedMessage = naza.decode(Serial.read());
    switch (decodedMessage) {
      case NazaDecoder::NAZA_MESSAGE_GPS:
        Serial.print("Latitude: ");
        Serial.println(naza.getLat(), 7);
        Serial.print("Longitude: ");
        Serial.println(naza.getLon(), 7);
        Serial.print("Altitude: ");
		Serial.println(naza.getGpsAlt(), 7);
        Serial.print("Fix: ");
        Serial.println(naza.getFixType());
        Serial.print("Satellites: ");
        Serial.println(naza.getNumSat());
        break;
      case NazaDecoder::NAZA_MESSAGE_COMPASS:
        Serial.print("Heading: ");
        Serial.println(naza.getHeading(), 2);
        break;
    }
  }
}
