//
//  NXTServo.cpp
//
//
//  Created by Sebastian Giles on 12/12/14.
//
//

#include "NXTServo.h"

NXTServo::NXTServo( const int &fwdPin, const int &revPin,
                    const int &interruptPin, const int &quadraturePin):
Motor(fwdPin, revPin, DEFAULT_VOLTAGE),
Encoder(interruptPin, quadraturePin)
{
    currentState = OFF;
    sampleTime = DEFAULT_SAMPLE_TIME;
    //Initial angle is established at this moment
    //so we can calculate the speed even on the first call of update()
    angleController.tune(AKP,AKI,AKD);
    speedController.tune(SKP,SKI,SKD);
    speedController.tolerate(1);
    angleController.tolerate(1);
    lastAngle=position();
    lastTime=millis();
    data.target=0;
}

boolean NXTServo::update(){

    if(millis()-lastTime<=sampleTime){
        return false;
    }

    long ms=millis();

    float angle = position();

    data.time = ms/1000.0;

    if(currentState==SPEED) {
        data.input = ((angle-lastAngle)/360.0) /
                        ((data.time-lastTime/1000.0)/60.0);

        Motor::turn(speedController.compute(data));

    } else if(currentState==ANGLE) {
        data.input=angle;
        Motor::turn(angleController.compute(data));
    }

    lastTime = ms;
    lastAngle = angle;

    return true;
}

float NXTServo::position(){
    long pos=read();
    return (float)pos/2.0;
}

void NXTServo::setSampleTime(const int &time){
    sampleTime=time;
}

void NXTServo::tuneSpeedPID(const float &kp, const float &ki, const float &kd){
    speedController.tune(kp,ki,kd);
}

void NXTServo::tuneAnglePID(const float &kp, const float &ki, const float &kd){
    angleController.tune(kp,ki,kd);
}


void NXTServo::turnAt(const float &speed){
    data.target=speed;
    currentState=SPEED;
}

void NXTServo::brake(){
    turnAt(0);
}

void NXTServo::goTo(const float &angle){
    data.target=angle;
    currentState=ANGLE;
}

void NXTServo::hold(){
    goTo(position());
}

void NXTServo::reset(){
    currentState=OFF;
    write(0);
    lastAngle=0;
    lastTime=millis();
    update();
}

void NXTServo::turn(const float &power){
    currentState=OFF;
    Motor::turn(power);
}

PIDData* NXTServo::getData(){
    return &data;
}
