/**
 * @file AltitudeSensor.h
 *
 * @brief Header file for AltitudeSensor class.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#ifndef _AltitudeSensor_H_
#define _AltitudeSensor_H_

#include "Arduino.h"
#include <SoftwareSerial.h>

class AltitudeSensor {
public:
	
	AltitudeSensor();

	void init();
	float getAltitude();
	void readAltitude();
	void printAltitude();

private:
	unsigned char USdata[4] = {};
    float altitude;

};

#endif