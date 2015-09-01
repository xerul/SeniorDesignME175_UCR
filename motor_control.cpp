#include "motor_control.h"

extern DigitalOut led1;
extern DigitalOut led2;
extern DigitalOut led3;
extern DigitalOut led4;
extern DigitalOut LeftDIR;
extern DigitalOut RightDIR;
extern PwmOut LeftHIGH;
extern PwmOut LeftLOW;
extern PwmOut RightHIGH;
extern PwmOut RightLOW;
extern Serial usb_pc;  
extern Serial bluetooth;
extern AnalogIn pot_right; //right
extern AnalogIn pot_left; //left

#define DEFAULT_SPEED        0.25
#define P2R_DIA_RATIO_L      3.54 //pitch to roll ratio of top frame
#define P2R_DIA_RATIO_R      4.10
#define ERROR_THRESHOLD      0.1
//#define KP                   0.013
//#define KI                   0.00001
//#define KD                   -0.05
#define KP                   0.012
#define KI                   0.00004
#define KD                   -0.05
#define MIN_SPEED_HIGH       0.075
#define MIN_SPEED_LOW        0.01
#define MAX_SPEED            0.99
#define MAX_RIGHT_BACK       0.28
#define MAX_RIGHT_FORWARD    0.1
#define MAX_LEFT_BACK        0.14
#define MAX_LEFT_FORWARD     0.53
#define CONST_MOVE_L         0.8
#define VEL_CONST            0.5

#define DIGITAL_FORWARD      1
#define DIGITAL_BACKWARD     0
 
/*Look up table
 *  0   lbs: kp 0.012, ki 0.00004, kd -0.05 
 *  150 lbs: kp 0.700, ki 0.00021, kd 0.0
*/

Motor::Motor()
{
    //constructor
   pos_ctr_R.kp = KP;
   pos_ctr_L.kp = KP;
   pos_ctr_R.ki = KI;
   pos_ctr_L.ki = KI;
   pos_ctr_R.kd = KD;
   pos_ctr_L.kd = KD;
   brakes = 1.0;    
}

void Motor::mod_PID(float a_roll) {
    
    //motor is not strong enoough to provide to torque,
    //increase the radius of the roll arms in the back.
    float const_roll = 0.000; //
    
    if (a_roll < 0.0) { a_roll = -a_roll;}
    pos_ctr_R.kp = (KP*a_roll*a_roll*a_roll*a_roll*const_roll + KP);
    pos_ctr_L.kp = (KP*a_roll*a_roll*a_roll*a_roll*const_roll + KP);
//    usb_pc.printf("kP:%4.3f,",pos_ctr_R.kp);
    
}

void Motor::set_fullbrake(MOT_COM side)
{
    /* PWM: 0 is hard break, 1 is coast */
    brakes = 1;
    if(side == LEFT) {LeftLOW.write(1.0);} 
    else if (side == RIGHT) {RightLOW.write(1.0);}
    else {LeftLOW.write(0.0); RightLOW.write(0.0);}
}

void Motor::set_fullcoast(MOT_COM side) {
    /* PWM: 0 is hard break, 1 is coast */
    brakes = 0;
    if(side == LEFT) {LeftLOW.write(0.0);} 
    else if (side == RIGHT) {RightLOW.write(0.0);}
    else {LeftLOW.write(1.0); RightLOW.write(1.0);} 
}

void Motor::set_direction(MOT_COM side, MOT_COM direction) {
    /* chooses the direction on motor driver to be forward or back*/

    if(side == LEFT) {
        if(direction == FORWARD) {
            LeftDIR = DIGITAL_FORWARD; //check to see if forward    
        }
        else {
            LeftDIR = DIGITAL_BACKWARD; //check to see if backward  
        }
    }
    else if(side == RIGHT) {
        if(direction == FORWARD) {
            RightDIR = DIGITAL_FORWARD; //check to see if forward    
        }
        else {
            RightDIR = DIGITAL_BACKWARD; //check to see if backward    
        }
    }
    else {
        //default case
        LeftDIR = DIGITAL_BACKWARD;
        RightDIR = DIGITAL_BACKWARD;
    }
}

void Motor::reset_PID() 
{
    pos_ctr_L.error = 0;
    pos_ctr_L.prev_err = 0;
    pos_ctr_L.sum = 0;
    
    pos_ctr_R.error = 0;
    pos_ctr_R.prev_err = 0;
    pos_ctr_R.sum = 0;
    
}

void Motor::stop_hard() {
    brakes = 1;
    set_fullbrake(LEFT);
    set_fullbrake(RIGHT);
    RightHIGH.write(0.0); 
    LeftHIGH.write(0.0);
}

void Motor::stop_soft() {
    brakes = 0;
    set_fullcoast(LEFT);
    set_fullcoast(RIGHT);
    RightHIGH.write(0.0); 
    LeftHIGH.write(0.0);
}

void Motor::move_to(Angles *actual, Angles *desired)  
{
    mCurrent.pitch = actual->pitch;
    mCurrent.roll = actual->roll;    

    mDesired.pitch = desired->pitch;
    mDesired.roll = desired->roll;    

    move_pitch_roll();
}

void Motor::move_motor(float move, MOT_COM side) {
    
    switch(side) {
         case RIGHT:
            RightHIGH.write(move);
            //if brakes on, PWML is HIGH, else coast
            (brakes) ? RightLOW.write(1.0) : RightLOW.write(move); 
//            bluetooth.printf("%f Right %d\r\n",move,RightDIR.read());
            break;
            
         case LEFT:
            LeftHIGH.write(move);
            //if brakes on, PWML is HIGH, else coast
            (brakes) ? LeftLOW.write(1.0) : LeftLOW.write(move); 
//            bluetooth.printf("%f Left %d\r\n",move,LeftDIR.read());
            break;

        default:
            RightHIGH.write(0.0);
            LeftHIGH.write(0.0);
            break;
    }
}


void Motor::move_pitch_roll() 
{
    float pid_right = 0, pid_left = 0;
    
    float diff_pitch = mCurrent.pitch - mDesired.pitch;
    float diff_roll = mCurrent.roll - mDesired.roll;
    
    //derivation in notebook
    float left_move = -((P2R_DIA_RATIO_L)*diff_pitch + diff_roll); //position calculation
    float right_move = -(P2R_DIA_RATIO_R)*diff_pitch + diff_roll;
    
    pid_right = pid_ctr(&pos_ctr_R , 0, right_move);
    pid_left = pid_ctr(&pos_ctr_L , 0, left_move); 
    
    if(pid_right > 0.0) {
        set_direction(RIGHT,FORWARD);
        pos_ctr_R.dir = DIGITAL_FORWARD; //FORWARD
        move_motor(pid_right,RIGHT);
    }
    else {
        set_direction(RIGHT,BACKWARD); 
        pos_ctr_R.dir = DIGITAL_BACKWARD;//BACKWARD
        move_motor(-pid_right,RIGHT);
    }
    
    if(pid_left > 0.0) {
        set_direction(LEFT,FORWARD); 
        pos_ctr_L.dir = DIGITAL_FORWARD;//FORWARD
        move_motor(pid_left,LEFT);
    }
    else {
        set_direction(LEFT,BACKWARD); 
        pos_ctr_L.dir = DIGITAL_BACKWARD;//BACKWARD
        move_motor(-pid_left,LEFT);
    }
    
}
//
//float Motor::read_pots(MOT_COM side)
//{
// //   if (side == RIGHT) { return(pot_right.read());}
////    else if (side == LEFT) { return(pot_left.read());}
////    else {return(0);}
////    
//return 0;
//}


float Motor::pid_ctr(PID *ctrl, float set, float actual) 
{
    float pid_control = 0;
    float Kp_ctr = 0;
    float Ki_ctr = 0;
    float Kd_ctr = 0;
    
    ctrl->prev_err = ctrl->error;
    ctrl->error = actual - set; //will always be positive
    ctrl->sum += ctrl->error;
    
    Kp_ctr = ctrl->kp*ctrl->error;
    Ki_ctr = ctrl->ki*ctrl->sum;
    Kd_ctr = ctrl->kd*(ctrl->error - ctrl->prev_err);

    pid_control = Kp_ctr + Ki_ctr + Kd_ctr;
    
    if(pid_control > MAX_SPEED) {pid_control = MAX_SPEED;}
    else if (pid_control < -MAX_SPEED) {pid_control = -MAX_SPEED;}
     
//    if(pid_control < MIN_SPEED_HIGH && pid_control > MIN_SPEED_LOW) {pid_control = MIN_SPEED_HIGH;}
    
//    usb_pc.printf("pi: %4.3f,%4.3f\r\n",Kp_ctr,Ki_ctr);
     
    return pid_control;
}

//void Motor::check_softstops()
//{
//    float left_check = read_pots(LEFT);
//    float right_check = read_pots(RIGHT);
//    
//    if(right_check>MAX_RIGHT_BACK || right_check<MAX_RIGHT_FORWARD ||
//        left_check<MAX_LEFT_BACK || left_check>MAX_LEFT_FORWARD) {
//        stop_hard();
//        led1 = 1;
//        led2 = 1;
//        led3 = 1;
//        led4 = 1;
//        bluetooth.printf("STOP! %4.3f %4.3f\r\n",left_check,right_check);
//        
//    }
//}

void Motor::tactile_stops(int a, int b, int c, int d) {
    if(a || b || c || d) {
        stop_hard();
    }
}

void Motor::velocity_control(Angles *curr, Angles *accel)
{
    static Angles prev;
    Angles deriv;
    float const_d = VEL_CONST;
    
    deriv.pitch =  const_d*(curr->pitch - prev.pitch);
    deriv.roll =   const_d*(curr->roll - prev.roll);
    
    prev = deriv;
    
    move_to(accel,&deriv);
    
}

void Motor::position_control(Angles *curr, Angles *accel)
{

    move_to(accel,curr);
    
}
//        void reset_pos();
//        void read_current();
//        void read_motor_pos();
//        






/*
    
    //map out how to move right and left actuators 
    //based on pitch and roll

*/