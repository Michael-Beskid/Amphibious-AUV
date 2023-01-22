/**
 * @file DepthSensor.h
 *
 * @brief Header file for DepthSensor class.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#ifndef _DEPTHSENSOR_H_
#define _DEPTHSENSOR_H_

#include "Arduino.h"
#include <Wire.h>
#include "MS5837/MS5837.h"

class DepthSensor {
public:

	DepthSensor();

	void init();
	float getDepth();
    void readDepth();
	void printDepth();

private:
	float depth;

};

#endif