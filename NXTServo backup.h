//
//  NXTServo.h
//
//  Everything regarding torque will be coded after everything else
//  is done, and a reliable way of calculating torque is found
//
//  Created by Sebastian Giles on 12/12/14.
//
//

#ifndef ____NXTServo__
#define ____NXTServo__

#include "Arduino.h"
#include "Motor.h"
#include "Encoder.h"

#define DEFAULT_SAMPLE_TIME 1
#define DEFAULT_VOLTAGE 9
#define MOTOR_DATA 51
#define TOLERANCE 0.5

struct MotorData{
    float time, setpoint, input, output, p,i,d;
};

class NXTServo : public Motor, public Encoder{
public:
    
    NXTServo(int fwdPin, int revPin, int interruptPin, int quadraturePin);
    
    //returns true if something changes meaning sampleTime is passed
    //should be run immediately after
    boolean update();
    
    void sampleTime(int time);
    
    int sampleTime();
    
    void supplyVoltage(float supplyVoltage);
    
    void tuneSpeedPID(float kp, float ki, float kd);
    
    void tuneAnglePID(float kp, float ki, float kd);
    
    void spinAt(float power);
    
    //Applies voltage to motor, returns true if voltage was possible
    void voltage(float voltage);
    
    //Gets motor cruising at specified speed
    void speed(float speed); //in rpm
    //stops the motor as fast as possible
    void brake();
    
    void coast();
    
    //Turns motor to given position
    void angle(float angle); //in degrees
    //holds current position
    void hold();
    //uses current position as new 0 reference point
    void reset();
    
    MotorData data();
    
private:
    
    MotorData data_;
    
    void applyVoltage(float voltage);
    
    enum mode{OFF, SPEED, ANGLE};

    mode controllerMode_;
    
    int sampleTime_;
    
    bool firstLoop_;
    
    float supplyVoltage_;
    
    float getAngle();
    
    float computePID(float input, float dTime,
                     float kp,    float ki,     float kd);
    
    float target_;
    
    long lastTime_;
    float lastAngle_; //used to compute speed
    float lastError_; //used to compute derivative of error
    float errorIntegral_;
    float lastVoltage_;
    
    //proportional, integral and derivative coefficients for
    //PID control of speed:
    float kps_;
    float kis_;
    float kds_;
    
    //same for angle:
    float kpa_;
    float kia_;
    float kda_;
    
};


#endif /* defined(____NXTServo__) */
