/**
 * @file QuadEncoder.h
 *
 * @brief Header file for QuadEncoder class.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#ifndef _QUADENCODER_H_
#define _QUADENCODER_H_

#include "Arduino.h"

class QuadEncoder {
public:
	
	QuadEncoder(int pinA, int pinB);
    
    void init();
    void updateCounter();
    float ballastControl(float setPoint); // TODO: Take in dt if you actually want to do PID control instead of P control. Later problem...
    void printEncoderInfo();
    
private:
    static const int TOTAL_COUNTS = 100;
    int ENCA, ENCB;
    int counter, state, lastState;  
    float error, error_prev, integral, integral_prev, derivative, control, PID;
    float setPoint_constrained;

    float Kp = 1.0;
    float Ki = 0.0;
    float Kd = 0.0;
    float i_limit = 0.0;
    const int dt = 0;

};

#endif