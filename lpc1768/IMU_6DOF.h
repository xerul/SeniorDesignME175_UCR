/* This library utilizes the L3G and LSM303D library 
*  with the MiniIMU-9 v3 motion sensor from pololu"
*  Written by Rex Lu for ME175, UCR Senior Design
*/

#ifndef IMU_6DOF_H
#define IMU_6DOF_H

#include "L3G.h"
#include "LSM303D.h"
#include <stdint.h>
#include <math.h>
#include "mbed.h"
#include "mbed_init.h"

#define INIT_TIME_MS               1000
#define PI                         3.1415926
#define CUTOFF_LOW                 5.0
#define CUTOFF_HIGH                1
#define GYRO_THRESHOLD             0.1
#define ACCEL_THRESHOLD            0.0001
#define ZERO_PITCH_DEG             0.0
#define ZERO_ROLL_DEG              0.0
#define WINDOW_SIZE                128


typedef struct xyz {
    float x,y,z;
    xyz() {x=0.0; y=0.0; z=0.0;}
} XYZ;  
        
        
class IMU_6DOF {
    
    public:
    
        XYZ actual_gyro;
        XYZ raw_gyro;
        XYZ actual_accel;
        XYZ raw_accel;
        float real_pitch;
        float real_roll;
        float pitch_g;
        float roll_g;
        float pitch_a;
        float roll_a;
        
        IMU_6DOF(void);
        void init_all(int ctr_rate_ms) ;
        void debug_on();
        void debug_off();
        void read_real_angles();
        void read_gyro();
        void read_raw_gyro();
        void read_gyro_angles();
        void read_accel();
        void read_raw_accel();
        void read_accel_angles();
        void complimentary_filter();
        void zero_gyro();
        void zero_accel();
        void print_data(XYZ *data) const;
        void print_end() const;
        void print_angles(); 
        void demean_data();
        
    private:
    
        float sensitivity_const_g;
        float sensitivity_const_a;
        int debug_print;
        float dT_sec;
        float init_pitch;
        float init_roll;
        float prev_pitch_g;
        float prev_roll_g;
        float window_gx[WINDOW_SIZE];
        float window_gy[WINDOW_SIZE];
         
        L3G gyro;
        LSM303 accel;
        Timer time;
        XYZ prev_accel;
        XYZ sum_g;
        XYZ pos_deg;
        XYZ prev_gyro;
        XYZ gyro_offset;
        
        void warm_up(int time_ms);
        XYZ low_pass(XYZ *curr, XYZ *prev);    
        XYZ high_pass(XYZ *curr, XYZ *prev);  
        float high_pass(float *curr, float *prev);    
        void set_gyro_accel_rate(int gyro_rate, int accel_Rate);
        void demean_data(float *data);
        void fix_gyroDrift();
    
};

 
#endif

