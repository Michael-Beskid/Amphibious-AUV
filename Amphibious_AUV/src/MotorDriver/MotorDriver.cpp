#include "MotorDriver.h"

const int MotorDriver::m1Pin = 0;
const int MotorDriver::m2Pin = 1;
const int MotorDriver::m3Pin = 2;
const int MotorDriver::m4Pin = 3;

const int MotorDriver::servo1Pin = 6;
const int MotorDriver::servo2Pin = 7;

PWMServo servo1;
PWMServo servo2;

MotorDriver::MotorDriver() {}

void MotorDriver::init() {
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  servo1.attach(servo1Pin, 900, 2100); //Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 900, 2100);

    // Arm servo channels
  servo1.write(0); // Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); // Set these to 90 for servos if you do not want them to briefly max out on startup. Keep these at 0 if you are using servo outputs for motors
  delay(5);

  // Arm OneShot125 motors
  m1_command_PWM = 125; // Command OneShot125 ESC from 125 to 250us pulse length
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  armMotors(); // Loop over commandMotors() until ESCs happily arm
}

void MotorDriver::setMotorCommands(float m1, float m2, float m3, float m4) {
  m1_command_scaled = m1;
  m2_command_scaled = m2;
  m3_command_scaled = m3;
  m4_command_scaled = m4;
}

void MotorDriver::setServoCommands(float s1, float s2) {
  s1_command_scaled = s1;
  s2_command_scaled = s2;
}

void MotorDriver::scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 
   * which are used to command the servos.
   */
  //Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled*125 + 125;
  m2_command_PWM = m2_command_scaled*125 + 125;
  m3_command_PWM = m3_command_scaled*125 + 125;
  m4_command_PWM = m4_command_scaled*125 + 125;
  //Constrain commands to motors within oneshot125 bounds
  m1_command_PWM = constrain(m1_command_PWM, 125, 250);
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);
  m3_command_PWM = constrain(m3_command_PWM, 125, 250);
  m4_command_PWM = constrain(m4_command_PWM, 125, 250);

  //Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);

}

void MotorDriver::commandMotors() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  
  //Write all motor pins high
  digitalWrite(m1Pin, HIGH);
  digitalWrite(m2Pin, HIGH);
  digitalWrite(m3Pin, HIGH);
  digitalWrite(m4Pin, HIGH);
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 4 ) { //Keep going until final (4th) pulse is finished, then done
    timer = micros();
    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((m3_command_PWM <= timer - pulseStart) && (flagM3==0)) {
      digitalWrite(m3Pin, LOW);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((m4_command_PWM <= timer - pulseStart) && (flagM4==0)) {
      digitalWrite(m4Pin, LOW);
      wentLow = wentLow + 1;
      flagM4 = 1;
    } 
  }
}

void MotorDriver::commandServos() {
  servo1.write(s1_command_PWM);
  servo2.write(s2_command_PWM);
}

void MotorDriver::armMotors() {
  //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*  
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void MotorDriver::throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
   * Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
   * minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function 
   * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
   * the motors to anything other than minimum value. Safety first. 
   */
    m1_command_PWM = 120;
    m2_command_PWM = 120;
    m3_command_PWM = 120;
    m4_command_PWM = 120;
}

void MotorDriver::printMotorCommands() {
  Serial.print(F("m1_command: "));
  Serial.print(m1_command_PWM);
  Serial.print(F(" m2_command: "));
  Serial.print(m2_command_PWM);
  Serial.print(F(" m3_command: "));
  Serial.print(m3_command_PWM);
  Serial.print(F(" m4_command: "));
  Serial.println(m4_command_PWM);
}

void MotorDriver::printMotorCommandsScaled() {
  Serial.print(F("m1_command: "));
  Serial.print(m1_command_scaled);
  Serial.print(F(" m2_command: "));
  Serial.print(m2_command_scaled);
  Serial.print(F(" m3_command: "));
  Serial.print(m3_command_scaled);
  Serial.print(F(" m4_command: "));
  Serial.println(m4_command_scaled);
}

void MotorDriver::printServoCommands() {
  Serial.print(F("s1_command: "));
  Serial.print(s1_command_PWM);
  Serial.print(F(" s2_command: "));
  Serial.print(s2_command_PWM);
}