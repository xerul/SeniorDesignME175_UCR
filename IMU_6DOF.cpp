/* This library utilizes the L3G and LSM303D library 
*  with the MiniIMU-9 v3 motion sensor from pololu"
*  Written by Rex Lu for ME175, UCR Senior Design
*/

#include "IMU_6DOF.h"
extern DigitalOut led1;
extern DigitalOut led2;
extern DigitalOut led3;
extern DigitalOut led4;

extern Serial usb_pc;  
extern Serial bluetooth;

// Constructors ///////////////////////////////////////////////////
IMU_6DOF::IMU_6DOF(void)
{
    
     prev_accel = actual_accel;
     prev_gyro = actual_gyro;
}

// PUBLICS /////////////////////////////////////////////////////////
void IMU_6DOF::init_all(int ctr_rate_ms) 
{
    dT_sec = ((float)ctr_rate_ms)/1000;
    
    if (!gyro.init())
    {
      (BLUETOOTH_ON) ? usb_pc.printf("Failed to autodetect gyro!\r\n")
        : bluetooth.printf("Failed to autodetect gyro!\r\n");
      while(1) {
            led1 = !led1; 
            led2 = !led2;
            led3 = !led3;
            led4 = !led4;
            wait(0.25);
      }
    }
    if(DEBUG_D) {(!BLUETOOTH_ON) ? usb_pc.printf("Gyro Enabled!\r\n") 
        : bluetooth.printf("Gyro Enabled!\r\n");}
        
    gyro.enableDefault();  
    
    wait(0.1);
    
    if (!accel.init())
    {
      (BLUETOOTH_ON) ? usb_pc.printf("Failed to autodetect accel!\r\n")
        : bluetooth.printf("Failed to autodetect accel!\r\n");
      while(1) {
            led1 = !led1; 
            led2 = !led2;
            led3 = !led3;
            led4 = !led4;
            wait(0.25);
      }
    }
    if(DEBUG_D) {(BLUETOOTH_ON) ? usb_pc.printf("Accel Enabled!\r\n") 
        : bluetooth.printf("Accel Enabled!\r\n");}
    accel.enableDefault();  
    
    //245,500,2000 dps, 2,4,6,8,16gs
    set_gyro_accel_rate((int) 500, (int) 6); 
    warm_up(INIT_TIME_MS);
    zero_gyro();
    zero_accel();
}


void IMU_6DOF::debug_on() 
{
    this->debug_print = 1;
}

void IMU_6DOF::debug_off() 
{
    this->debug_print = 0;
}

void IMU_6DOF::read_real_angles()
{
    read_accel_angles();
    read_gyro_angles();
     
}

void IMU_6DOF::read_gyro(void) 
{
        
    read_raw_gyro();
    prev_gyro = actual_gyro;
    this->actual_gyro.x = raw_gyro.x - gyro_offset.x;
    this->actual_gyro.y = raw_gyro.y - gyro_offset.y;
    this->actual_gyro.z = raw_gyro.z - gyro_offset.z;
    
   // actual_gyro = low_pass(&actual_gyro,&prev_gyro);
}

void IMU_6DOF::read_raw_gyro(void) 
{
    gyro.read();
    raw_gyro.x = ((float) gyro.g.x)*sensitivity_const_g;
    raw_gyro.y = ((float) gyro.g.y)*sensitivity_const_g;
    raw_gyro.z = ((float) gyro.g.z)*sensitivity_const_g;
}

void IMU_6DOF::read_gyro_angles(void) 
{
        
        read_gyro();
    
        if(actual_gyro.x < GYRO_THRESHOLD && actual_gyro.x > -GYRO_THRESHOLD) actual_gyro.x=0.0;
        if(actual_gyro.y < GYRO_THRESHOLD && actual_gyro.y > -GYRO_THRESHOLD) actual_gyro.y=0.0;
        if(actual_gyro.z < GYRO_THRESHOLD && actual_gyro.z > -GYRO_THRESHOLD) actual_gyro.z=0.0;
           
        sum_g.x += ((prev_gyro.x+actual_gyro.x)/2.0)*dT_sec;
        sum_g.y += ((prev_gyro.y+actual_gyro.y)/2.0)*dT_sec;
        sum_g.z += ((prev_gyro.z+actual_gyro.z)/2.0)*dT_sec;

        pitch_g = sum_g.x;
        roll_g = sum_g.y;    
        fix_gyroDrift();
 //       pitch_g = high_pass(&pitch_g, &prev_pitch_g);   
//        roll_g = high_pass(&roll_g, &prev_roll_g);   
//        prev_pitch_g = pitch_g;
//        prev_roll_g = roll_g;
//        demean_data(&pitch_g);
            
}

void IMU_6DOF::read_accel(void) 
{
    read_raw_accel();
    prev_accel = actual_accel;
//    this->actual_accel.x = raw_accel.x*sensitivity_const_a;
//    this->actual_accel.y = raw_accel.y*sensitivity_const_a;
//    this->actual_accel.z = raw_accel.z*sensitivity_const_a;
    actual_accel = low_pass(&raw_accel,&prev_accel);
}

void IMU_6DOF::read_raw_accel(void) 
{
    accel.read();
    this->raw_accel.x = ((float) accel.a.x)*sensitivity_const_a;
    this->raw_accel.y = ((float) accel.a.y)*sensitivity_const_a;
    this->raw_accel.z = ((float) accel.a.z)*sensitivity_const_a;
}

void IMU_6DOF::read_accel_angles()
{
    read_accel();
        
    if(actual_accel.x < ACCEL_THRESHOLD && actual_accel.x > -ACCEL_THRESHOLD) actual_accel.x=0.0;
    if(actual_accel.y < ACCEL_THRESHOLD && actual_accel.y > -ACCEL_THRESHOLD) actual_accel.y=0.0;
    if(actual_accel.z < (-1.0+ACCEL_THRESHOLD) && actual_accel.z > (-1.0-ACCEL_THRESHOLD)) actual_accel.z=-1.0;
        
    float temp_y = actual_accel.y*actual_accel.y + actual_accel.z*actual_accel.z;
    float temp_x = actual_accel.x*actual_accel.x + actual_accel.z*actual_accel.z; 
//    pitch_a = atan(actual_accel.y/actual_accel.z)*180.0/PI;   
    pitch_a = atan2(-actual_accel.y,sqrt(temp_x))*180.0/PI; 
    pitch_a += -1.5; //pitch offset
    roll_a = atan2(-actual_accel.x,sqrt(temp_y))*180.0/PI;
    roll_a = -roll_a; //direction of chip
    roll_a += -2.5; //roll offset
}

void IMU_6DOF::zero_gyro()
{
    /* Read gyro 100x to get gyro_offset by averaging.
     * Argument must be the control loop time of the 
     * main, such that gyro is sampled at same rate.
     */
     
    if(DEBUG_D) {(!BLUETOOTH_ON) ? usb_pc.printf("Zeroing Gyro ...\r\n") 
        : bluetooth.printf("Zeroing Gyro ...\r\n");}
        
    float sum_gx = 0;
    float sum_gy = 0;
    float sum_gz = 0;
    int num = 100;
    
    time.start();
    
    //wait to calibrate
    for (int i=0; i<num; i++) {
        time.reset();
        gyro.read();
        sum_gx += gyro.g.x*sensitivity_const_g;
        sum_gy += gyro.g.y*sensitivity_const_g;
        sum_gz += gyro.g.z*sensitivity_const_g;        
        while(time.read_ms() < dT_sec) {wait_us(1);}   
    }
    
    time.stop();
    
    gyro_offset.x = sum_gx/num; //using gyro_offset->x fails here ...
    gyro_offset.y = sum_gy/num;
    gyro_offset.z = sum_gz/num;      
    
    usb_pc.printf("%f\r\n",dT_sec);
}


void IMU_6DOF::zero_accel()
{
    if(DEBUG_D) {(!BLUETOOTH_ON) ? usb_pc.printf("Zeroing Accel ...\r\n") 
        : bluetooth.printf("Zeroing Accel ...\r\n");}
        
    float sum_pitch = 0;
    float sum_roll = 0;
    int num = 100;
    
    time.start();
    
    //wait to calibrate
    for (int i=0; i<num; i++) {
        time.reset();
        read_accel_angles();
        sum_pitch += pitch_a;
        sum_roll += roll_a;     
        while(time.read_ms() < dT_sec) {wait_us(1);}   
    }
    init_pitch = sum_pitch/num; 
    init_roll = sum_roll/num;
}

void IMU_6DOF::print_data(XYZ *data) const 
{
    if (!BLUETOOTH_ON) {
        usb_pc.printf(" %4.3f %4.3f %4.3f",data->x,data->y,data->z);
    }
    else {
        bluetooth.printf(" %4.3f %4.3f %4.3f",data->x,data->y,data->z);
    }
}

void IMU_6DOF::print_end() const
{
    if (!BLUETOOTH_ON) {
        usb_pc.printf(";\r\n");
    }
    else {
        bluetooth.printf(";\r\n");
    }
}

void IMU_6DOF::print_angles(void) 
{
    if (!BLUETOOTH_ON) {
        usb_pc.printf("%4.3f,%4.3f,%4.3f,%4.3f,%4.f,%4.f\r\n",roll_a,pitch_a,roll_g,pitch_g,real_roll,real_pitch);
    }
    else {
        bluetooth.printf("a: %4.3f %4.3f, g: %4.3f %4.3f,%4.f,%4.f\r\n",roll_a,pitch_a,roll_g,pitch_g,real_roll,real_pitch);
    }
}

void IMU_6DOF::demean_data(float *new_data)
{
    float sum_window = 0;
    
    for (int i=0; i<WINDOW_SIZE-2; i++) {
        window_gx[i]=window_gx[i+1];   
        sum_window += window_gx[i];
    }           
    window_gx[WINDOW_SIZE-1] = *new_data;
    *new_data = (sum_window+(*new_data))/WINDOW_SIZE;
    
}

////PRIVATES//////////////////////////////////////////////

void IMU_6DOF::warm_up(int time_ms) 
{
    /* warm up IMU by running reads for some time*/
    if(DEBUG_D) {(!BLUETOOTH_ON) ? usb_pc.printf("Warming up IMU ...\r\n") 
        : bluetooth.printf("Warming up IMU ...\r\n");}
        
    for (int i=0; i<time_ms; i++) {
        gyro.read();
        accel.read(); 
        wait(0.001);
    }
}

XYZ IMU_6DOF::low_pass(XYZ *curr, XYZ *prev) 
{
    /*1st order low pass IIR Filter*/
    
   float RC = 1.0/(2.0*PI*CUTOFF_LOW);
   float a = dT_sec/(RC + dT_sec);
   XYZ temp;
   temp.x = a*curr->x + (1.0-a)*prev->x;
   temp.y = a*curr->y + (1.0-a)*prev->y;
   temp.z = a*curr->z + (1.0-a)*prev->z;

   return temp;
}

XYZ IMU_6DOF::high_pass(XYZ *curr, XYZ *prev) 
{
    /*1st order high pass IIR Filter*/
    
   float RC = 1.0/(2.0*PI*CUTOFF_HIGH);
   float a = RC/(RC + dT_sec);
   XYZ temp;
   //α * y[i-1] + α * (x[i] - x[i-1])
   temp.x = a*prev->x + a*(curr->x - prev->x);
   temp.y = a*prev->y + a*(curr->y - prev->y);
   temp.z = a*prev->z + a*(curr->z - prev->z);

   return temp;
}

float IMU_6DOF::high_pass(float *curr,float *prev) 
{
    /*1st order high pass IIR Filter*/
    
   float RC = 1.0/(2.0*PI*CUTOFF_HIGH);
   float a = RC/(RC + dT_sec);
   float temp;
   //α * y[i-1] + α * (x[i] - x[i-1])
   temp = a*(*prev) + a*((*curr) - (*prev));

   return temp;
}


void IMU_6DOF::complimentary_filter()
{
    float alpha = 0.90;
    real_pitch = alpha*pitch_g + (1.0-alpha)*pitch_a;
    real_roll = alpha*roll_g + (1.0-alpha)*roll_a;    
}

void IMU_6DOF::set_gyro_accel_rate(int gyro_rate, int accel_rate)
{
    /* Chooses the setting for gyro & accel max angular rates 
     * Refer to data sheets for numbers. 
     *
     * FS = 00 (+/- 250 dps full scale) (L3GD20H is 245)
     * FS = 01 (+/- 500 dps full scale)
     * FS = 10 (+/- 2000 dps full scale)
     * AFS = 000 (+/- 2 g full scale)
     * AFS = 001 (+/- 4 g full scale)
     * AFS = 010 (+/- 6 g full scale)
     * AFS = 011 (+/- 8 g full scale)
     * AFS = 100 (+/- 16 g full scale)
     */
    (!BLUETOOTH_ON) ? (usb_pc.printf("Gyro: %d dps, Accel: %d g\r\n",gyro_rate,accel_rate))
        : (bluetooth.printf("Gyro: %d dps, Accel: %d g\r\n",gyro_rate,accel_rate));
    
    switch(gyro_rate) {
        //Sensitivity CONSTANT in degrees/sec for gyro
        case(245):
            gyro.writeReg(gyro.CTRL_REG4,(0x00<<4));
            sensitivity_const_g = 0.00875; //from data sheet
//            sensitivity_const_g /= 1.169592; //tested coefficient
            break;
            
        case(500):
            gyro.writeReg(gyro.CTRL_REG4,(0x01<<4));
            sensitivity_const_g = 0.01750; //from data sheet
//            sensitivity_const_g /= 1.1468; //tested coefficient
            break;
            
        case(2000):
            gyro.writeReg(gyro.CTRL_REG4,(0x02<<4));
            sensitivity_const_g = 0.070; //from data sheet
//            sensitivity_const_g /= 1.1468; //tested coefficient
            break;
            
        default:
            gyro.writeReg(gyro.CTRL_REG4,0x00);
            sensitivity_const_g = 0.00875; //245dps
//            sensitivity_const_g /= 1.169592; //tested coefficient
            break;
    }
    
    switch(accel_rate) {
        //Sensitivity CONSTANT in g's for acceleration
        case(2):
            accel.writeReg(accel.CTRL2,0x00);
            sensitivity_const_a = 0.000061;
            break;
            
        case(4):
            accel.writeReg(accel.CTRL2,(0x01<<3));
            sensitivity_const_a = 0.000122;
            break;
            
        case(6):
            accel.writeReg(accel.CTRL2,(0x02<<3));
            sensitivity_const_a = 0.000183;
            break;
            
        case(8):
            accel.writeReg(accel.CTRL2,(0x03<<3));
            sensitivity_const_a = 0.000244;
            break;
            
        case(16):
            accel.writeReg(accel.CTRL2,(0x04<<3));
            sensitivity_const_a = 0.000732;
            break;
            
        default:
            accel.writeReg(accel.CTRL2,0x00);
            sensitivity_const_a = 0.000061; // +/-2g
            break;
    }   
}

void IMU_6DOF::fix_gyroDrift()
{
    static unsigned long count = 0; //for constant values 
    static float temp=0;
    time.start();
    if (temp*0.90 < fabs(pitch_a) && temp*1.10 > fabs(pitch_a)) {
        count++;    
    }
    
    if(time.read_ms() > 500) {
         temp = pitch_a;
         time.reset();
         time.stop();
    }
    
    if(count >= 100) {
        pitch_g = pitch_a;
        temp = pitch_a;
        time.reset();
        count = 0;
        time.stop();
    }
}
