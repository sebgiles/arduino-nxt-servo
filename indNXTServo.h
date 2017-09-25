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

#define DEFAULT_SAMPLE_TIME 10
#define DEFAULT_VOLTAGE 9

#define SPEEDKP 0
#define SPEEDKI 0
#define SPEEDKD 0

#define ANGLEKP 0
#define ANGLEKI 0
#define ANGLEKD 0

class NXTServo : public Motor, private Encoder{
public:
    
    NXTServo(int fwdPin, int revPin, int interruptPin, int quadraturePin);
    
    //returns true if something changes meaning sampleTime has passed
    //should be run very frequently
    bool update();
    
    //takes integer value in millis
    void setSampleTime(int newSampleTime);
            
    void tuneSpeedPID(float kp, float ki, float kd);
    
    void tuneAnglePID(float kp, float ki, float kd);
        
    //Gets motor cruising at specified speed
    void turnAt(float speed); //in rpm

    //stops the motor as fast as possible
    void brake();
    
    //Turns motor to given position
    void goTo(float angle); //in degrees

    //holds current position
    void hold();

    //return current position in degrees
    float position();

    //uses current position as new 0 reference point
    void reset();

    //override is necessasry to disable pid
    void turn(float power);

    //returns struct from PID object
    PIDData* getData();
        
private:
    
    PID speedController;
    PID angleController;

    PIDData data;

    enum mode {OFF, SPEED, ANGLE};

    mode currentState = OFF;
    
    int sampleTime = DEFAULT_SAMPLE_TIME;
        
    long lastTime; //in millis, used to check if updating is needed
    float lastAngle; //both are used to compute speed
    
};


#endif /* defined(____NXTServo__) */