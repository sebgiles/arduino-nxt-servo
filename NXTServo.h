//
//  NXTServo.h
//
//
//  Created by Sebastian Giles on 12/12/14.
//
//

#ifndef ____NXTServo__
#define ____NXTServo__

#include "Arduino.h"
#include "Motor.h"
#include "PID.h"
#include "Encoder.h"

#define DEFAULT_SAMPLE_TIME 20
#define DEFAULT_VOLTAGE 9

#define SKP 0
#define SKI 0
#define SKD 0

#define AKP 0
#define AKI 0
#define AKD 0

class NXTServo : public Motor, public PID, private Encoder{
public:

  NXTServo(const int &white, const int &black, const int &yellow, const int &blue);

  //returns true if something changes meaning sampleTime has passed
  //should be run very frequently
  bool update();

  //takes integer value in millis
  void setSampleTime(const int &newSampleTime);

  void tuneSpeedPID(const float &kp, const float &ki, const float &kd);

  void tuneAnglePID(const float &kp, const float &ki, const float &kd);

  //Gets motor cruising at specified speed
  void turnAt(const float &speed); //in rpm

  //stops the motor as fast as possible
  void brake();

  //Turns motor to given position
  void goTo(const float &angle); //in degrees

  //holds current position
  void hold();

  //return current position in degrees
  float position();

  //uses current position as new 0 reference point
  void reset();

  //override is necessasry to disable pid
  void turn(const float &power);

  //returns struct from PID object
  PIDData* getData();

private:

  PID speedController;
  PID angleController;

  PIDData data;

  enum mode {OFF, SPEED, ANGLE};

  mode currentState;

  unsigned int sampleTime;

  unsigned long lastTime; //in millis, used to check if updating is needed
  float lastAngle; //both are used to compute speed

};


#endif /* defined(____NXTServo__) */
