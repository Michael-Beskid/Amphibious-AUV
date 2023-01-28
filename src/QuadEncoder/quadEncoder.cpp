/**
 * @file QuadEncoder.cpp
 *
 * @brief QuadEncoder class contains all of the functions for position control of the ballast cylinders.
 * 
 * The quadrotor vehicle features two syringes for the ballast system. Each cylinder has a quadrature shaft encoder
 *   coupled to the shaft of a continuous rotation servo to measure the linear translation of the piston within the cylinder.
 * 
 * Wiring:
 *   Encoder 1:
 *     Channel A : pin 9
 *     Channel B : pin 10
 *   Encoder 2:
 *     Channel A : pin 11
 *     Channel B : pin 12
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#include "QuadEncoder.h"

/**
 * @brief Constructor
 * 
 * @param pinA Channel A pin.
 * @param pinB Channel B pin.
 */
QuadEncoder::QuadEncoder(int pinA, int pinB) {
    ENCA = pinA;
    ENCB = pinB;
}

/**
 * @brief Initialize quadrature encoder.
 */
void QuadEncoder::init() {
  pinMode (ENCA,INPUT);
  pinMode (ENCB,INPUT);
  lastState = digitalRead(ENCA);
}

/**
 * @brief Update the counter for the quadrature encoder.
 * 
 * Reads the inputs from the A and B channels of the encoder to detect any rotation and updates the counter appropriately.
 */
void QuadEncoder::updateCounter() {
  state = digitalRead(ENCA);
  
  if (state != lastState){     
    if (digitalRead(ENCB) != state) { 
      counter ++;
    } else {
      counter --;
    }
  } 

  lastState = state;
}

/**
 * @brief Calculate the speed commands to send to the ballast system servos.
 * 
 * Computes a control output to send to the ballast system servo motors based upon the error between
 *   the target positions and the current positions measured by the encoders.
 * 
 * @returns a normalized scaled servo speed command [0-1]. A value of 0.5 corresponds to stopped.
 */
float QuadEncoder::ballastControl(float setPoint) {
  setPoint_constrained = (constrain(setPoint, 0, 100)/100.0)*TOTAL_COUNTS;
  error = setPoint_constrained - counter;
  integral = integral_prev + error*dt;
  integral = constrain(integral, -i_limit, i_limit);
  derivative = (error - error_prev)/dt; 
  PID = 0.02*(Kp*error + Ki*integral - Kd*derivative);
  control = 0.5 - constrain(PID, -1.0, 1.0)*0.5;
  return control;
}

/**
 * @brief Print the servo commands to the Serial monitor.
 */
void QuadEncoder::printEncoderInfo() {
  Serial.print(F("Counter value: "));
  Serial.print(counter);
  Serial.print(F(" Goal value: "));
  Serial.print(setPoint_constrained);
  Serial.print(F(" Scaled motor command: "));
  Serial.println(control);
}