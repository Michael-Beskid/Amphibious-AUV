#include "TrackingCamera.h"

TrackingCamera::TrackingCamera() {}

void TrackingCamera::init() {
    Serial2.begin(9600);
}

float TrackingCamera::getPosX() {
  return posX;
}

float TrackingCamera::getPosY() {
  return posY;
}

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

void TrackingCamera::printPosition() {
  Serial.print(F("X-Position: "));
  Serial.print(posX);
  Serial.print(F(" m     Y-Position: "));
  Serial.print(posY);
  Serial.println(F(" m"));
}