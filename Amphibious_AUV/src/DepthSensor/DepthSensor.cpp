#include "DepthSensor.h"

MS5837 ms5837;

DepthSensor::DepthSensor() {}

void DepthSensor::init() {
  
  Wire1.begin();

  if (ms5837.init() == false) {
    Serial.println("MS5837 initialization unsuccessful");
    Serial.println("Check wiring or try cycling power");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    while(1) {}
  }
  
  ms5837.setModel(MS5837::MS5837_30BA);
  ms5837.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

float DepthSensor::getDepth() {
  return depth;
}

void DepthSensor::readDepth() {
  ms5837.read();
  depth = ms5837.depth();
}

void DepthSensor::printDepth() {
	Serial.print(F("Depth: "));
	Serial.print(depth);
	Serial.println(F(" m"));
}