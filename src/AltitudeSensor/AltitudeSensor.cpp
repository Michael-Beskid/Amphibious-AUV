#include "AltitudeSensor.h"

SoftwareSerial USSerial(15, 14); // RX, TX

AltitudeSensor::AltitudeSensor() {}

void AltitudeSensor::init() {
   USSerial.begin(9600);
}

float AltitudeSensor::getAltitude() {
	return altitude;
}

void AltitudeSensor::readAltitude() {
  
  do {
     for (int i=0; i<4; i++) {
       USdata[i] = USSerial.read();
     }
  } while (USSerial.read() == 0xff);

  USSerial.flush();

  if (USdata[0] == 0xff) {
      int sum;
      float distance;
      sum = (USdata[0] + USdata[1] + USdata[2]) & 0x00FF;     
      if (sum == USdata[3]) {
        distance = (USdata[1] << 8) + USdata[2];
        if (distance > 30) {
           altitude = distance;
        }
      }
   }
}

void AltitudeSensor::printAltitude() {
   Serial.print(F("Altitude: "));
   Serial.print(altitude/10.0);
   Serial.println(F(" cm"));
}