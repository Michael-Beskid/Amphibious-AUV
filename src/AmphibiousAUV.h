/**
 * @file AmphibiousAUV.h
 *
 * @brief Header file for main Amphibious.cpp file.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#include <Arduino.h> 

// General declarations
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
unsigned int slowLoopCounter;
bool blinkAlternate;

// Normalized desired state
float thro_des, roll_des, pitch_des, yaw_des;

// Controller variables
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;
float error_altitude, error_altitude_prev, altitude_des_prev, integral_altitude, integral_altitude_prev, derivative_altitude, altitude_PID = 0;
float error_posX, error_posY, posX_control, posY_control = 0;

// Flight modes enumeration
enum flightModes {
  MANUAL,
  AUTONOMOUS
};

enum flightModes flightMode;

// Manual states enumeration
enum manualStates {
  MANUAL_STARTUP,
  NORMAL,
};

enum manualStates manualState;

// Autonomous states enumeration
enum autoStates {
  AUTO_STARTUP, // Initilaization
  TAKEOFF,      // Take off vertically from start position to an altitude of 1.5 meters
  FORWARD,      // Translate forward a distance of 5.0 meters
  LAND,         // Land on the ground surface
  STOP          // End autonomous mission
};

enum autoStates missionState;

// Motion planning
const float POS_DB_RADIUS = 0.25; // Deadband radius for evaluating reached position targets
boolean motorsOff = false;
boolean underwater = false;
float altitude_des = 0.0; // mm
float depth_des = 0.0;
float target_posX = 0.0; // meters
float target_posY = 0.0; // meters

// Function declarations
void printDebugInfo();
void getFlightMode();
void calibrateESCs();
void controlMixer();
void getDesState();
void getDesStateAuto();
void getDesStateManual();
void controlANGLE();
void throttleCut();
void setTargetAltitude(float alt);
void setTargetDepth(float depth);
void setTargetPos(float posX, float posY);
boolean reachedTarget();
void loopRate(int freq);
void loopBlink();
void setupBlink(int numBlinks,int upTime, int downTime);
void printDesiredState();
void printPIDoutput();
void printFlightMode();
void printLoopRate();
void ISR();