# NXTServo  
Arduino library to handle Lego Mindstorms NXT servo motors.
Features precise position control and speed control.

Relies on the PJRC Encoder library, install separetely: https://www.pjrc.com/teensy/td_libs_Encoder.html

Each motor requires 4 pins on Arduino:  
2 digital inputs for the encoder signal  
    (at least one should be an interrupt pin, more info: https://www.pjrc.com/teensy/td_libs_Encoder.html#polling)  
2 pwm outputs  

H-bridge (L293 etc.) or motor shield is required for powering the motor with 9V. Used channels must be enabled, pwm is sent directly to the control pins. No extra wiring or circuits are necessary.

Please refer to this table: http://www.personal.psu.edu/jpm5375/MotorPinout.png  
Pins 1 and 2 are connected to the H-bridge/shield output  
Pins 5 and 6 carry the encoder signal to the arduino  
Pin 3 is GND  
Pin 4 should be +4.5V to power the optical encoder, I use 5V without any problem, do at your own risk  
