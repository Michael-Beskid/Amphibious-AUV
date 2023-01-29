/**
 * @file GPS.cpp
 *
 * @brief GPS class to interface with the NEO-M9N GPS module.
 * 
 * This class utilizes the Sparkfun ublox library tp interface with the GPS module.
 * 
 * Wiring:
 *   Red: +5V
 *   Black: GND
 *   Blue: SDA (pin 17)
 *   Purple: SCL (pin 16)
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#include "GPS.h"

SFE_UBLOX_GPS myGPS;

GPS::GPS() {}

/**
 * @brief Initialize the GPS.
 */
void GPS::init() {
  
    Wire1.begin();

    if (myGPS.begin(Wire1) == false) { //Connect to the Ublox module using Wire port
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
    }

    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.saveConfiguration(); //Save the current settings to flash and BBR
    
}

void GPS::setStartPos() {
    initLatitude = myGPS.getLatitude();
    initLongitude = myGPS.getLongitude();
}

/**
 * @brief Read the most recent latitude and longitude from the GPS module
 */
void GPS::read() {
    latitude = myGPS.getLatitude();
    longitude = myGPS.getLongitude();

    posX = (latitude - initLatitude)*metersPerDegreeLat*0.0000001;
    posY = (longitude - initLongitude)*metersPerDegreeLong*0.0000001;
}

/**
 * @brief Get the X-position of the quadrotor in [m] relative to the starting position
 * 
 * The vehicle doesn't actually know it's heading... so this is just taken to be North.
 * 
 * @param posX X-posiiton of the quadrotor in meters.
 */
float GPS::getPosX() {
    return posX;
}

/**
 * @brief Get the Y-position of the quadrotor in [m] relative to the starting position
 * 
 * The vehicle doesn't actually know it's heading... so this is just taken to be East.
 * 
 * @param posY Y-posiiton of the quadrotor in meters.
 */
float GPS::getPosY() {
    return posY;
}

/**
 * @brief Print the (X,Y) position of the quadrotor from GPS data
 */
void GPS::printPosition() {
	Serial.print(F("X-Position: "));
	Serial.print(posX);
	Serial.print(F(" m"));
    Serial.print(F("  Y-Position: "));
	Serial.print(posY);
	Serial.println(F(" m"));
}