/**
 * @file AltitudeSensor.cpp
 *
 * @brief Driver for A02YYUW waterproof ultrasonic rangefinder.
 * 
 * This driver library is based off sample code provided by DFRobot.
 * 
 * Wiring:
 *   Red: +5V
 *   Black: GND
 *   Green: RX (pin 15)
 *   Blue: TX (pin 14)
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#include "AltitudeSensor.h"

// Software Serial port for ultrasonic sensor
SoftwareSerial USSerial(15, 14); // RX, TX

AltitudeSensor::AltitudeSensor() {}

/**
 * @brief Initialize the ultrasonic sensor.
 */
void AltitudeSensor::init() {
   USSerial.begin(9600);
}

/**
 * @brief Get the current altitude.
 *
 * @returns most recent altitude reading in milimeters.
 */
float AltitudeSensor::getAltitude() {
   return altitude;
}

/**
 * @brief Read the ultrasonic sensor.
 *
 *  Reads the latest altitude measurement from the sensor
 *    and updates the current stored altitude value.
 */
void AltitudeSensor::readAltitude() {

   do {
      for (int i = 0; i < 4; i++) {
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

/**
 * @brief Print the current altitude in [cm] to the Serial monitor.
 */
void AltitudeSensor::printAltitude() {
   Serial.print(F("Altitude: "));
   Serial.print(altitude / 10.0);
   Serial.println(F(" cm"));
}