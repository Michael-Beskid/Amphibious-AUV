/**
 * @file DepthSensor.cpp
 *
 * @brief DepthSensor class to interface with the Blue Robotics Bar30 pressure sensor.
 * 
 * This class utilizes the MS5837 library created by Blue Roboitcs to interface with the Bar30 pressure sensor.
 *   Note that this library was modified slighlty to use the "Wire1" I2C port instead of the standard "Wire." 
 * 
 * Wiring:
 *   Red: +5V
 *   Black: GND
 *   White: SDA (pin 17)
 *   Green: SCL (pin 16)
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#include "DepthSensor.h"

MS5837 ms5837;

DepthSensor::DepthSensor() {}

/**
 * @brief Initialize the depth sensor.
 */
void DepthSensor::init() {
  
  // Brgin I2C communication
  Wire1.begin();

  // Check for succesful initialization response
  if (ms5837.init() == false) {
    Serial.println("MS5837 initialization unsuccessful");
    Serial.println("Check wiring or try cycling power");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    while(1) {}
  }
  
  ms5837.setModel(MS5837::MS5837_30BA);
  ms5837.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

/**
 * @brief Get the current depth.
 *
 * @returns most recent depth reading in milimeters.
 */
float DepthSensor::getDepth() {
  return depth;
}

/**
 * @brief Read the ultrasonic sensor.
 *
 *  Reads the latest depth measurement from the sensor
 *    and updates the current stored depth value.
 */
void DepthSensor::readDepth() {
  ms5837.read();
  depth = ms5837.depth();
}

/**
 * @brief Print the current depth in [m] to the Serial monitor.
 */
void DepthSensor::printDepth() {
	Serial.print(F("Depth: "));
	Serial.print(depth);
	Serial.println(F(" m"));
}