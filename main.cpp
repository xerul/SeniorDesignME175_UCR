/*
    By Rex Lu, June 2015, ME175
    This software works with the linux evtest for communicating with the Logitech Steering Wheel and Pedals.
    The motor control algorithm is tuned for the 12v 23Amp DC motors and the 6dof IMU accelerometer and gyro.
*/

#include "mbed.h"
#include "mbed_init.h"
#include "task_control.h"

/*TODO:
 * Tune controller using linear velocity profile
 * Fix ramp and step response
 * Run tests with different users and weights
 * Build lookup table for weight vs PID
 * Continue with velocity control through serial
*/

int main() {
//    main loop
    task_scheduler();

}
