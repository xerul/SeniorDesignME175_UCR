#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "mbed.h"
#include "mbed_init.h"


typedef enum MOTOR_COMMMANDS {
    /* Motor commands */
    STOP=0,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    NUM_MOT_COM
} MOT_COM;

typedef struct angles {
    float pitch;    
    float roll;

    angles() {pitch = 0; roll = 0; }
    
} Angles;

typedef struct control {
    float error;
    float prev_err;
    float kp;
    float ki;
    float kd;
    float sum;
    int dir; 
    control() {error=0; prev_err=0; kp=0; ki=0; kd=0; sum=0; dir=0;}
} PID;

class Motor {
    public:
        float brakes;
        
        Motor(void);
        void mod_PID(float a_roll);
        void set_fullbrake(MOT_COM side);
        void set_fullcoast(MOT_COM side);
        void set_direction(MOT_COM side, MOT_COM directions); 
        void reset_PID();
        void stop_soft();
        void stop_hard();
        void move_to(Angles *actual, Angles *desired);  
        float read_left_pot();
//        float read_pots(MOT_COM side);
        void check_softstops();
        void tactile_stops(int a, int b, int c, int d);
        void velocity_control(Angles *curr, Angles *accel);
        void position_control(Angles *curr, Angles *accel);
//        void reset_pos();
//        void read_current();
//        void read_motor_pos();
//        void velocity_prof();
        
    private:   
        
        Angles mCurrent;
        Angles mDesired;
        PID pos_ctr_L;
        PID pos_ctr_R;
        float pid_ctr(PID *ctrl, float set, float actual);
        void move_pitch_roll();
        void move_motor(float move, MOT_COM side);
};









#endif

