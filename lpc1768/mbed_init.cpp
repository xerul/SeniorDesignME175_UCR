#include "mbed_init.h"

//LEDs on the mbed
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

//Send/Transmit Data 
Serial usb_pc(USBTX, USBRX);  
Serial bluetooth(p13, p14);

//IMU accel + gyro bus
I2C i2c(p28, p27);

//Motor Control Drivers
DigitalOut LeftDIR(p30);
DigitalOut RightDIR(p29);
PwmOut LeftHIGH(p22);
PwmOut LeftLOW(p21);
PwmOut RightHIGH(p24);
PwmOut RightLOW(p23);

//play music debug
PwmOut speaker(p25);

//AnalogIn pot_right(p19); //right
//AnalogIn pot_left(p20); //left

//User Buttons
DigitalIn BtnStart(p7);
DigitalIn BtnReset(p6);
DigitalIn BtnStop(p8);

//Tactile Switches
DigitalIn sw_FR(p9);
DigitalIn sw_BR(p10);
DigitalIn sw_FL(p11);
DigitalIn sw_BL(p12);


