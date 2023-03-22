/**
 * @file ControllerVariables.h
 *
 * @brief Contains controller variables and PID gains for attitude, altitude, depth, and position controllers.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

// General controller variables
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

// Stabilization controller variables
float i_limit = 25.0;         //Integrator saturation level, mostly for safety (default 25.0)
float Kp_roll_angle = 0.2;    //Roll P-gain
float Ki_roll_angle = 0.3;    //Roll I-gain
float Kd_roll_angle = 0.05;   //Roll D-gain
float Kp_pitch_angle = 0.2;   //Pitch P-gain
float Ki_pitch_angle = 0.3;   //Pitch I-gain
float Kd_pitch_angle = 0.05;  //Pitch D-gain
float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain

// Altitude controller variables
float hover_throttle = 0.525;       //Baseline throttle for hovering
float Kp_altitude = 1.0;            //Altitude P-gain
float Ki_altitude = 0.2;            //Altitude I-gain
float Kd_altitude = 0.5;            //Altitude D-gain
float i_limit_altitude = 10000.0;   //Integrator saturation level

// Depth controller variables
// TODO: Tune these PID gains and integral saturation limit
float Kp_depth = 1.0;           //Altitude P-gain
float Ki_depth = 0.0;           //Altitude I-gain
float Kd_depth = 0.0;           //Altitude D-gain
float i_limit_depth = 100.0;    //Integrator saturation level

// Position controller variables
float Kp_position = 0.1;  // Full angle at 10m away from target