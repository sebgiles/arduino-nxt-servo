//
//  NXTServo.cpp
//  
//
//  Created by Sebastian Giles on 12/12/14.
//
//

#include "NXTServo.h"

NXTServo::NXTServo(int fwdPin, int revPin, int interruptPin, int quadraturePin):
Motor(fwdPin, revPin),
Encoder(interruptPin, quadraturePin){
    
    
    tuneAnglePID(0,0,0);
    tuneSpeedPID(0,0,0);
    
    //default state
    controllerMode_=OFF;
    
    sampleTime_=DEFAULT_SAMPLE_TIME;
    supplyVoltage_=DEFAULT_VOLTAGE;
    
    //Initial angle is established at this moment
    lastAngle_=getAngle();
    lastTime_=millis();
    lastVoltage_=0;
}

boolean NXTServo::update(){
    long time = millis();
    if(time-lastTime_<=sampleTime_){
        return false;
    }
    
    float angle=getAngle();
    
    PIDvalues vals;
    
    if(controllerMode_==SPEED) {
        float speed=(angle-lastAngle_)/((float)(time-lastTime_)/1000.0)*60/360;
        applyVoltage(speedController_(target_, input, time, vals));
        
    } else if(controllerMode_==ANGLE) {
        
        applyVoltage(computePID(angle, dTime, kpa_, kia_, kda_));
        
    } else if(controllerMode_==OFF) {
        data_.output=lastVoltage_;
        data_.input=0;
        data_.setpoint=0;
        data_.p=0;
        data_.i=0;
        data_.d=0;
    }
    data_.time=(float)time/1000.0;
    
    lastTime_ = time;
    lastAngle_ = angle;

    return true;
}

float NXTServo::getAngle(){
    long pos=read();
    return (float)pos/2.0;
}

void NXTServo::sampleTime(int time){
    sampleTime_=time;
}

void NXTServo::tuneSpeedPID(float kp, float ki, float kd){
    speedController_.tune(kp,ki,kd);
}

void NXTServo::tuneAnglePID(float kp, float ki, float kd){
    angleController_.tune(kp,ki,kd);
}

void NXTServo::voltage(float voltage){
    controllerMode_=OFF;
    applyVoltage(voltage);
}

void NXTServo::supplyVoltage(float supplyVoltage){
    supplyVoltage_=supplyVoltage;
}

void NXTServo::spinAt(float power){
    controllerMode_=OFF;
    applyVoltage(power*supplyVoltage_);
}

void NXTServo::applyVoltage(float voltage){
    if      (voltage >  supplyVoltage_)
        voltage =   supplyVoltage_;
    
    else if (voltage < -supplyVoltage_)
        voltage =  -supplyVoltage_;
    
    lastVoltage_=voltage;
    Motor::spinAt(voltage/supplyVoltage_);
}

void NXTServo::speed(float speed){
    target_=speed;
    controllerMode_=SPEED;
}

void NXTServo::brake(){
    speed(0);
}

void NXTServo::coast(){
    spinAt(0);
}

void NXTServo::angle(float angle){
    target_=angle;
    controllerMode_=ANGLE;
}

void NXTServo::hold(){
    angle(getAngle());
}

void NXTServo::reset(){
    controllerMode_=OFF;
    write(0);
    lastAngle_=0;
    lastTime_=millis();
    update();
}

int NXTServo::sampleTime(){
    return sampleTime_;
}

MotorData NXTServo::data(){
    return data_;
}

