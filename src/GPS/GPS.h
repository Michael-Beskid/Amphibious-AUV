/**
 * @file GPS.h
 *
 * @brief Header file for GPS class.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#ifndef _GPS_H_
#define _GPS_H_

#include "Arduino.h"
#include <Wire.h>
#include <SparkFun_Ublox_Arduino_Library.h>

class GPS {
public:
	
	GPS();

    void init();
    void read();
    void setStartPos();
    float getPosX();
    float getPosY();
    void printPosition();

private:
    static const int metersPerDegreeLat = 111319; // Constant for any longitude
    static const int metersPerDegreeLong = 82376; // Multiplied by a factor of cos(latitude). Chose 0.74 for cos(42.275) for WPI Worcester, MA.
    int latitude, longitude;
    int initLatitude, initLongitude;
    float posX, posY;

};

#endif