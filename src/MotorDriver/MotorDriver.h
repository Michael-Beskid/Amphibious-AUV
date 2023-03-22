/**
 * @file MotorDriver.h
 *
 * @brief Header file for MotorDriver class.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#ifndef _MOTORDRIVER_H_
#define _MOTORDRIVER_H_

#include "Arduino.h"
#include <PWMServo.h>

class MotorDriver {
public:
	
	MotorDriver();
    
    void init();
    void setMotorCommands(float m1, float m2, float m3, float m4);
    void setServoCommands(float m1, float m2);
    void scaleCommands();
    void commandMotors();
    void commandServos();
    void armMotors();
    void throttleCut();
    void printMotorCommands();
    void printMotorCommandsScaled();
    void printServoCommands();

private:
    static const int m1Pin, m2Pin, m3Pin, m4Pin;
    static const int servo1Pin, servo2Pin;
    float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
    int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;
    float s1_command_scaled, s2_command_scaled;
    int s1_command_PWM, s2_command_PWM;

};

#endif