/**
 * @file AmphibiousAUV.cpp
 *
 * @brief Flight computer for amphibious AUV.
 * 
 * Flight control software development for an autonmous quadrotor vehicle that is capable of combined at-will aerial
 *   and underwater operation. The vehicle can be operated in MANUAL and AUTONOMOUS modes. This project is a Major
 *   Qualifying Project (MQP) compelted in partial fulfillment of degrees in Aerospace Engineering and Robotics
 *   Engineering at Worcester Polytechnic Institute (WPI).
 * 
 * Team Members: Michael Beskid, Ryan Brunelle, Calista Carrignan, Robert Devlin, Toshak Patel, & Kofi Sarfo.
 * 
 * The project is hosted on GitHub: https://github.com/Michael-Beskid/Amphibious-AUV.
 * 
 * Acknowdegments:
 *   This project is adapted from the dRehmFlight VTOL flight controller created my Nicholas Rehm. Thank you Nick
 *      for making your exellent work publicly avaialble to enable the development of cool proejct such as this one. 
 *      dRehmFlight VTOL on GitHub: https://github.com/nickrehm/dRehmFlight.
 *   Thank you to project advisors Michael Demetriou (AE Department) and Loris Fichera (RBE Department).
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#include <Arduino.h>                        // Arduino library
#include <Wire.h>                           // I2C communication
#include <SPI.h>                            // SPI communication
#include <SoftwareSerial.h>                 // Serial communication
#include "AmphibiousAUV.h"                  // General variables and function declarations
#include "ControllerVariables.h"            // Controller variables
#include "MotorDriver/MotorDriver.h"        // Motor and servo commands
#include "IMU/IMU.h"                        // MPU 6050 IMU (6-axis accel/gyro)
#include "RadioComm/RadioComm.h"            // Radio communication
#include "DepthSensor/DepthSensor.h"        // BlueRobotics Bar30 depth sensor
#include "AltitudeSensor/AltitudeSensor.h"  // A02YYUW waterproof ultrasonic rangefinder
#include "TrackingCamera/TrackingCamera.h"  // Intel RealSense T265 tracking camera

MotorDriver motors;
RadioComm radio;
IMU imu;
DepthSensor depthSensor;
AltitudeSensor altitudeSensor;
TrackingCamera camera;                                      

/**
 * @brief Perform setup tasks before entering the main flight control loop.
 * 
 * This is the Arduino built-in setup() function. setup() is used to establish
 *   a USB Serial connection, initilaize communication with sensors, set state
 *   variables to initial conditions, begin receiving radio data, and arm the 
 *   electronic speed controllers (ESCs) for the motors.
 * 
 * The functions "calcualte_IMU_error()" and "Calibrate_ESCs" may be uncommented
 *   for calibration purposes but should ordinarily remain commented out.
 */
void setup() {

  // Begin USB Serial
  Serial.begin(500000);

  // Initialize sensors
  altitudeSensor.init();
  //depthSensor.init();
  camera.init();
  imu.init();
  delay(500);

  // Set built in LED to turn on to signal startup
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Start in manual flight mode
  flightMode = MANUAL;
  manualState = MANUAL_STARTUP;

  motorsOff = false;
  underwater = false;

  delay(5);

  // Initilaize motor driver
  motors.init();

  // Initialize radio communication
  radio.init();
  delay(20);
  //Attach interrupt and point to corresponding ISR function
  attachInterrupt(digitalPinToInterrupt(radio.getPPMpin()), ISR, CHANGE);

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calculate_IMU_error();

  // Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  // PROPS OFF. Code will not proceed past this if uncommented.
  //calibrateESCs(); 
  
  // Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); // numBlinks, upTime (ms), downTime (ms)

}

/**
 * @brief Main flight control loop.
 * 
 * This is the Arduino built-in loop() function. The loop runs continuously after the 
 *   setup() function is executed. The main flight control loop performs a number of tasks
 *   such as reading sensor data, computing state estimates, managing the mission state, 
 *   calculating control outputs, and commanding the motors and servos.
 */                                              
void loop() {
  // Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); // Indicate we are in main loop with short blink every 1.5 seconds

  // Print data at 100hz (uncomment one at a time below for troubleshooting)
  printDebugInfo();

  // Pull raw gyro, accelerometer, and magnetometer data from IMU and LP filter to remove noise
  imu.readData();
  // Update roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  imu.Madgwick(imu.getGyroX(), -imu.getGyroY(), -imu.getGyroZ(), -imu.getAccX(), imu.getAccY(), imu.getAccZ(), dt); 

  // Get vehicle position from tracking camera
  camera.recvSerial();
  camera.readData(); 
  
  // Get altitude/depth (sampling at 10 Hz bc can't handle 2 kHz)
  slowLoopCounter++;
  if (slowLoopCounter == 200) { 
    if (underwater) {
      depthSensor.readDepth();
    } else {
      altitudeSensor.readAltitude();
    }
    slowLoopCounter = 0;
  }

  // Get the current flight mode
  getFlightMode();

  // Compute desired state
  getDesState(); //Convert raw commands to normalized values based on saturated control limits
  
  // PID controller for attitude stabilization
  controlANGLE(); //Stabilize on angle setpoint

  // Actuator mixing and scaling to PWM values
  controlMixer(); // Mix PID outputs to scaled actuator commands
  motors.scaleCommands(); // Scale motor and servo commads

  // Throttle cut check
  throttleCut(); // Directly set motor commands to low if ch5 is high

  // Command actuators
  motors.commandMotors(); // Send command pulses to each motor pin using OneShot125 protocol
  motors.commandServos(); // Write PWM value to servo object
    
  // Get vehicle commands for next loop iteration
  radio.getCommands(); // Pull current available radio commands
  radio.failSafe(); // Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  // Regulate loop rate
  loopRate(2000); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

/**
 * @brief Prints useful information to the Serial monitor for debugging purposes.
 * 
 * Any of the lines below can be uncommented to print the corresponding values to
 *   the Serial monitor for troublehsooting. Only one line should be uncommented
 *   at a time to ensure that the print statements will not slow down the main
 *   flight control loop and impact the vehicle's performance.
 */
void printDebugInfo() {
  if (current_time - print_counter > 10000) {
      print_counter = micros();
      //radio.printData();
      //printDesiredState();
      //printPIDoutput();
      //printFlightMode();
      //motors.printMotorCommands();
      //motors.printMotorCommandsScaled();
      //motors.printServoCommands();
      //imu.printGyroData();
      //imu.printAccelData();
      //imu.printRollPitchYaw();
      //altitudeSensor.printAltitude();
      //depthSensor.printDepth();
      //camera.printPosition();
      //printLoopRate();
    }
}

/**
 * @brief Get the current flight mode from the radio transmitter
 */
void getFlightMode() {
  if (radio.getPWM(6) > 1500) {
    flightMode = AUTONOMOUS;
  } else {
    flightMode = MANUAL;
  }
}

/**
 * @brief Used in to allow standard ESC calibration procedure with the radio to take place.
 * 
 * From dRehmFlight:
 *   Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
 *   power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
 *   uncommented when performing an ESC calibration.
 */
void calibrateESCs() {
   while (true) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0;
    
      digitalWrite(13, HIGH); //LED on to indicate we are not in main loop

      radio.getCommands(); //Pulls current available radio commands
      radio.failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      imu.readData(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
      imu.Madgwick(imu.getGyroX(), -imu.getGyroY(), -imu.getGyroZ(), -imu.getAccX(), imu.getAccY(), imu.getAccZ(), dt);  //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      
      motors.setMotorCommands(thro_des, thro_des, thro_des, thro_des); //Sets all 4 motors to match throttle input

      motors.scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
      
      motors.commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
      
      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}

/**
 * @brief Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
 *   in preparation to be sent to the motor ESCs and servos.
 * 
 * From dRehmFlight:
 *   Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
 *   vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
 *   should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
 *   normalized (0 to 1) thro_des command for throttle control.
 *   in preparation to be sent to the motor ESCs and servos.
 */
void controlMixer() {
   
  // Motor mixing
  float m1 = thro_des - pitch_PID - roll_PID - yaw_PID; //Front right
  float m2 = thro_des + pitch_PID + roll_PID - yaw_PID; //Back left
  float m3 = thro_des + pitch_PID - roll_PID + yaw_PID; //Back right
  float m4 = thro_des - pitch_PID + roll_PID + yaw_PID; //Front left 
  motors.setMotorCommands(m1, m2, m3, m4);

  //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  float s1 = 0;
  float s2 = 0;
  motors.setServoCommands(s1, s2);
 
}

/**
 * @brief TODO
 */

// The outer state machine allows for toggling between flight modes, control of which is mapped to the radio transmitter
// The inner state machines for each flight mode are controlled by the computer
// The manual mode begins in STARTUP to set a few variables before transitioning into NORMAL for manual radio control
// The auto mode begins in STARTUP to set a few variables, then transitions between a sequence of pre-defined states to complete the autonomous mission
void getDesState() {

  switch (flightMode) {

    case MANUAL:
      switch (manualState) {
        case MANUAL_STARTUP:
          motorsOff = false;
          missionState = AUTO_STARTUP; // Reset AUTO mode state machine
          manualState = NORMAL;
          break;
        case NORMAL:
          break;
        default:
          break;
      }

      getDesStateManual();
      break;

    case AUTONOMOUS: 
      switch (missionState) {
        case AUTO_STARTUP:
          integral_altitude_prev = 0.0;
          error_altitude_prev = 0.0;
          motorsOff = false;
          manualState = MANUAL_STARTUP; // Reset MANUAL mode state machine
          missionState = TAKEOFF;      
          setTargetAltitude(1.5);
          setTargetPos(0.0, 0.0);
          break;
        case TAKEOFF:
          if (reachedTarget()) {
            missionState = FORWARD;
            setTargetPos(5.0, 0.0);
          }
          break;
        case FORWARD:
          if (reachedTarget()) {
            missionState = LAND;
            setTargetAltitude(0.0);
          }
          break;
        case LAND:
          if (reachedTarget()) {
            missionState = STOP;
            motorsOff = true;
          }
          break;
        case STOP:
          break;
        default:
          break;
      }  

      getDesStateAuto();
      break;

    default:
      getDesStateManual();
      break;
  }
}

/**
 * @brief TODO
 */
void getDesStateAuto() {

  // PID Altitude Controller
  error_altitude = altitude_des - altitudeSensor.getAltitude();
  integral_altitude = integral_altitude_prev + error_altitude*dt;
  integral_altitude = constrain(integral_altitude, -i_limit_altitude, i_limit_altitude); //Saturate integrator to prevent unsafe buildup
  derivative_altitude = (error_altitude - error_altitude_prev)/dt; 
  altitude_PID = 0.00005*(Kp_altitude*error_altitude + Ki_altitude*integral_altitude - Kd_altitude*derivative_altitude); //Scaled by .00005 to bring within 0 to 1 range

  // Update variables
  integral_altitude_prev = integral_altitude;
  error_altitude_prev = error_altitude;

  // Proportional Position Controller
  error_posX = target_posX - camera.getPosX();
  error_posY = target_posY - camera.getPosY();
  posX_control = Kp_position*error_posX;
  posY_control = Kp_position*error_posY;

  // Set desired throttle value from altitude controller
  thro_des = hover_throttle + altitude_PID;

  // TODO: Check conventions to confirm that X and Y are correctly mapped to pitch and roll axes

  // Set desired roll and pitch angles from position controller
  roll_des = posY_control ; //Between -1 and 1
  pitch_des = posX_control; //Between -1 and 1
  yaw_des = 0;
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  
}

/**
 * @brief Normalizes desired control values to appropriate values
 * 
 * From dRehmFlight:
 *   Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
 *   RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
 *   roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
 *   (rate mode). yaw_des is scaled to be within max yaw in degrees/sec.
 */
void getDesStateManual() {
  
  thro_des = (radio.getPWM(1) - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (radio.getPWM(2) - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (radio.getPWM(3) - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (radio.getPWM(4) - 1500.0)/500.0; //Between -1 and 1
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  
}

/**
 * @brief Computes control commands based on state error (angle)
 * 
 * From dRehmFlight:
 *   Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
 *   getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
 *   are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
 *   excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
 *   they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
 *   if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
 *   terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
 *   can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
 */
void controlANGLE() {
  
  //Roll
  error_roll = roll_des - imu.getRoll();
  integral_roll = integral_roll_prev + error_roll*dt;
  if (radio.getPWM(1) < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = imu.getGyroX();
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - imu.getPitch();
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (radio.getPWM(1) < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = imu.getGyroY();
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - imu.getGyroZ();
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (radio.getPWM(1) < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
  
}

/**
 * @brief Directly set actuator outputs to minimum value if throttle cut is triggered.
 * 
 * From dRehmFlgiht:
 *   Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum if channel 5 is high.
 *   This is the last function called before commandMotors() is called so that the last thing checked is if the user is giving
 *   permission to command the motors to anything other than minimum value. Safety first.
 */
void throttleCut() {
  if (radio.getPWM(5) > 1500 || motorsOff) {
    motors.throttleCut();
  }
}

/**
 * @brief Set target altitude.
 * 
 * @param alt Desired altitude in meters.
 */
void setTargetAltitude(float alt) {
  altitude_des = alt*1000; // convert meters to mm
}

/**
 * @brief Set target depth.
 * 
 * @param depth Desired depth in meters.
 */
void setTargetDepth(float depth) {
  depth_des = depth;
}

/**
 * @brief Set target (X,Y) position.
 * 
 * @param posX Desired X-position in meters.
 * @param posY Desired Y-position in meters.
 */
void setTargetPos(float posX, float posY) {
  target_posX = posX;
  target_posY = posY;
}

/**
 * @brief Checks whether the vehicle has reached its target location.
 * 
 * During aerial operation, this function checks whether both the vehicle's
 *   (X,Y) position and altitude are within a tolerance of the target values.
 *   During underwater operation, this function checks whether both the vehicle's
 *   (X,Y) position and depth are within a toelrance of the the target values.
 * 
 * The global variable POS_DB_RADIUS can be modified to change the tolerance 
 *   required to consider a target position "reached."
 * 
 * @returns 'true' if vehicle has reached target. 
 */
boolean reachedTarget() {
  return abs(target_posX - camera.getPosX()) < POS_DB_RADIUS 
    && abs(target_posY - camera.getPosY()) < POS_DB_RADIUS
    && abs(altitude_des - altitudeSensor.getAltitude()) < POS_DB_RADIUS;
}

/**
 * @brief Regulate main loop rate to specified frequency.
 * 
 * From dRehmFlight:
 *   It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
 *   background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
 *   the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
 *   be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
 *   and remain above 2kHz, without needing to retune all of our filtering parameters.
 * 
 * @param freq Loop frequency in Hz.
 */
void loopRate(int freq) {

  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

/**
 * @brief Blink LED on board to indicate main loop is running.
 */
void loopBlink() {
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //Pin 13 is built in LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

/**
 * @brief Simple function to make LED on board blink as desired
 */
void setupBlink(int numBlinks,int upTime, int downTime) {
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

/**
 * @brief Print desired vehicle state to the Serial monitor.
 * 
 * The desired state incudes the desired throttle, deisred roll, 
 *   desired pitch, and desired yaw values.
 */
void printDesiredState() {
  Serial.print(F("thro_des: "));
  Serial.print(thro_des);
  Serial.print(F(" roll_des: "));
  Serial.print(roll_des);
  Serial.print(F(" pitch_des: "));
  Serial.print(pitch_des);
  Serial.print(F(" yaw_des: "));
  Serial.println(yaw_des);
}

/**
 * @brief Print PID output values to the Serial monitor.
 * 
 * This includes PID outputs for roll, pitch, and yaw from stabilization controller.
 */
void printPIDoutput() {
  Serial.print(F("roll_PID: "));
  Serial.print(roll_PID);
  Serial.print(F(" pitch_PID: "));
  Serial.print(pitch_PID);
  Serial.print(F(" yaw_PID: "));
  Serial.println(yaw_PID);
}

/**
 * @brief Print the current flight mode to the Serial monitor.
 */
void printFlightMode() {
  Serial.print(F("Flight Mode: "));
  if (flightMode) {
    Serial.println(F("Autonomous"));
  } else {
    Serial.println(F("Manual"));
  }
}

/**
 * @brief Print the loop rate to the Serial monitor.
 */
void printLoopRate() {
  Serial.print(F("dt = "));
  Serial.println(dt*1000000.0);
}

/**
 * @brief Interupt service rotine for reading radio commands.
 */
void ISR() {
  radio.getPPM();
}