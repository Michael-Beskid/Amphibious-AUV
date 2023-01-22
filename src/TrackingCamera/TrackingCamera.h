/**
 * @file TrackingCamera.h
 *
 * @brief Header file for TrackingCamera class.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#ifndef _TrackingCamera_H_
#define _TrackingCamera_H_

#include "Arduino.h"

class TrackingCamera {
public:
	
	TrackingCamera();

    void init();
    float getPosX();
    float getPosY();
	void readData();
	void recvSerial();
    void printPosition();

private:
	float posX;
    float posY;
    uint8_t upperX;
    uint8_t lowerX;
    uint8_t upperY;
    uint8_t lowerY;
    boolean newPosData = false;

};

#endif