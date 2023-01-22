/**
 * @file TrackingCamera.cpp
 *
 * @brief Class for interfacing with the Intel RealSense T265 tracking camera.
 * 
 * Wiring:
 *   Unspecified: RX (pin 7)
 *   Unspecified: TX (pin 8)
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */
#include "TrackingCamera.h"

TrackingCamera::TrackingCamera() {}

/**
 * @brief Initialize the tracking camera.
 */
void TrackingCamera::init() {
    Serial2.begin(9600);
}

/**
 * @brief Get the current X-position.
 *
 * @returns most recent X-position in meters.
 */
float TrackingCamera::getPosX() {
  return posX;
}

/**
 * @brief Get the current Y-position.
 *
 * @returns most recent Y-position in meters.
 */
float TrackingCamera::getPosY() {
  return posY;
}

/**
 * @brief Read position data from the tracking camera.
 *
 * If available, reads the latest position estimate from the camera
 *   and updates the current (X,Y) position. 
 */
void TrackingCamera::readData() {
    if (newPosData == true) {
      int posX = (upperX << 8) + lowerX;
      if(posX > 0x8000) posX = 0xFFFF - posX;
      posX = float(posX)/1000.0;
      int posY = (upperY << 8) + lowerY;
      if(posY > 0x8000) posY = -(0xFFFE - posY);
      posY = float(posY)/1000.0;
      newPosData = false;
    }
}

/**
 * @brief Receive and process Serial data from the camera.
 */
void TrackingCamera::recvSerial() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  uint8_t inByte;

  while (Serial2.available() > 0 && newPosData == false) {
    inByte = Serial2.read();

    if (recvInProgress == true) {
      switch (ndx) {
        case 0:
          upperX = inByte;
          ndx++;
          break;
        case 1:
          lowerX = inByte;
          ndx++;
          break;
        case 2:
          upperY = inByte;
          ndx++;
          break;
        case 3:
          lowerY = inByte;
          recvInProgress = false;
          newPosData = true;
          ndx = 0;
          break;
      }
    }
    else if (inByte == startMarker) {
      recvInProgress = true;
    }
  }
}

/**
 * @brief Print the current (X,Y) position in [m] to the Serial monitor.
 */
void TrackingCamera::printPosition() {
  Serial.print(F("X-Position: "));
  Serial.print(posX);
  Serial.print(F(" m     Y-Position: "));
  Serial.print(posY);
  Serial.println(F(" m"));
}