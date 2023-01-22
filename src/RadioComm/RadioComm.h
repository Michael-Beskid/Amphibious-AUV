/**
 * @file RadioComm.h
 *
 * @brief Header file for RadioComm class.
 *
 * @author Michael Beskid
 * Contact: mjbeskid@wpi.edu
 *
 */

#ifndef _RADIOCOMM_H_
#define _RADIOCOMM_H_

#include "Arduino.h"

class RadioComm {
public:
	
	RadioComm();

	void init();
	int getPPMpin();
	unsigned long getPWM(int ch_num);
	void setFailSafe();
	void failSafe();
	void getCommands();
	void printData();
	void getPPM();

private:
    const int PPM_Pin = 23;
	unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
	unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
	unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
    unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
    int ppm_counter = 0;
    unsigned long time_ms = 0;

	//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
	unsigned long channel_1_fs = 1000; //thro
	unsigned long channel_2_fs = 1500; //ail
	unsigned long channel_3_fs = 1500; //elev
	unsigned long channel_4_fs = 1500; //rudd
	unsigned long channel_5_fs = 2000; //greater than 1500 = throttle cut
	unsigned long channel_6_fs = 1000; //less than 1500 = MANUAL flight mode

	/** Gets distance measurment from ultrasonic sensor
	 */
	unsigned long readRadioPWM(int ch_num);

};

#endif