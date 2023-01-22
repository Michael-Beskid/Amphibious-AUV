/**
 * @file RadioComm.cpp
 *
 * @brief RadioComm class contains all of the functions needed for radio communication 
 * 
 * The radio receiver on the quadrotor is a PPM type receiver.
 * 
 * Wiring:
 *   Radio RX PPM pin: pin 23
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#include "RadioComm.h"

RadioComm::RadioComm() {}

/**
 * @brief Initialize radio communication.
 */
void RadioComm::init() {
  pinMode(PPM_Pin, INPUT_PULLUP); // Declare interrupt pin
  delay(20);
}

/**
 * @brief Initialize radio communication.
 * 
 * @returns PPM pin number.
 */
int RadioComm::getPPMpin() {
  return PPM_Pin;
}

/**
 * @brief Get the most recent radio PWM value from the specified channel.
 * 
 * @param ch_num Channel number to read PWM value from.
 * @returns PWM value from the specified channel.
 */
unsigned long RadioComm::getPWM(int ch_num) {
  unsigned long returnPWM = 0;
  
  if (ch_num == 1) {
    returnPWM = channel_1_pwm;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_pwm;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_pwm;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_pwm;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_pwm;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_pwm;
  }
  
  return returnPWM;
}

/**
 * @brief Read current radio commands from interrupt service routine.
 * 
 * @param ch_num Channel number to read radio command from.
 * @returns raw radio command from specified channel.
 */
unsigned long RadioComm::readRadioPWM(int ch_num) {

  unsigned long returnPWM = 0;
  
  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }
  
  return returnPWM;
}

/**
 * @brief Interupt service routine for reading PPM radio signals from transmitter.
 */
void RadioComm::getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig==1) { //Only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    
    if (dt_ppm > 5000) { //Waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }
  
    if (ppm_counter == 1) { //First pulse
      channel_1_raw = dt_ppm;
    }
  
    if (ppm_counter == 2) { //Second pulse
      channel_2_raw = dt_ppm;
    }
  
    if (ppm_counter == 3) { //Third pulse
      channel_3_raw = dt_ppm;
    }
  
    if (ppm_counter == 4) { //Fourth pulse
      channel_4_raw = dt_ppm;
    }
  
    if (ppm_counter == 5) { //Fifth pulse
      channel_5_raw = dt_ppm;
    }
  
    if (ppm_counter == 6) { //Sixth pulse
      channel_6_raw = dt_ppm;
    }
    
    ppm_counter = ppm_counter + 1;
  }
}

/**
 * @brief Set radio channels to default (safe) values.
 */
void RadioComm::setFailSafe() {
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;
}

/**
 * @brief Set all commands to default values if poor radio commadns are received.
 * 
 * From dRehmFlight:
 *   Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
 *   the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
 *   connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
 *   channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
 *   your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
 */
void RadioComm::failSafe() {

  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}

/**
 * @brief Get raw PWM values for every channel from the radio.
 * 
 * From dRehmFlight:
 *   Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
 *   the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from 
 *   this one which is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the values
 *   are pulled from the SBUS library directly. The raw radio commands are filtered with a first order low-pass filter to eliminate
 *   any really high frequency noise. 
 */
void RadioComm::getCommands() {

  // Get values from radio channels
  channel_1_pwm = readRadioPWM(1);
  channel_2_pwm = readRadioPWM(2);
  channel_3_pwm = readRadioPWM(3);
  channel_4_pwm = readRadioPWM(4);
  channel_5_pwm = readRadioPWM(5);
  channel_6_pwm = readRadioPWM(6);
  
  // Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
  
}

/**
 * @brief Print radio values to the Serial monitor.
 */
void RadioComm::printData() {
  Serial.print(F(" CH1: "));
  Serial.print(channel_1_pwm);
  Serial.print(F(" CH2: "));
  Serial.print(channel_2_pwm);
  Serial.print(F(" CH3: "));
  Serial.print(channel_3_pwm);
  Serial.print(F(" CH4: "));
  Serial.print(channel_4_pwm);
  Serial.print(F(" CH5: "));
  Serial.print(channel_5_pwm);
  Serial.print(F(" CH6: "));
  Serial.println(channel_6_pwm);
}