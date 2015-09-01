/*Main Loop*/
#include "mbed.h"
#include "task_control.h"

extern DigitalOut led1;
extern DigitalOut led2;
extern DigitalOut led3;
extern DigitalOut led4;

extern Serial usb_pc;  
extern Serial bluetooth;
extern PwmOut speaker;

extern DigitalIn BtnStart;
extern DigitalIn BtnStop;
extern DigitalIn BtnReset;
extern DigitalIn sw_FR;
extern DigitalIn sw_BR;
extern DigitalIn sw_FL;
extern DigitalIn sw_BL;


#define MAX_PITCH                   3.0
#define MIN_PITCH                   -5.0
#define MAX_ROLL                    5.0
#define MIN_ROLL                    -5.0
#define MAX_STEER                   16298.0
#define MIN_STEER                   0.0
#define MAX_PEDALS                  255
#define MIN_PEDALS                  -255
#define PEDALS_CENTER               2081.0
#define INIT_PITCH                  0.0
#define INIT_ROLL                   0.0
#define TEST_PITCH                  5.0
#define TEST_ROLL                   0.0

#define MIN_ZERO                    -0.5
#define MAX_ZERO                    0.5
#define CTR_TIMER_MSEC              10
#define SM1_TIME                    CTR_TIMER_MSEC
#define SM2_TIME                    CTR_TIMER_MSEC*200
#define CTR_INCREMENT               0.5
#define CTR_INTERVAL                1

Ticker isr_task;
IMU_6DOF imu6;
Motor mot;
Timer runtime;

volatile int timer_flag = 0; // isr_timer() sets this to 1
int start_flag = 0;
int move_flag = 0;
int time_count = 0;

char pedalsData[33] = "";          
char steerData[33] = "";
int data_flag = 0;
int data_index = 0;
int steer = 0;
int pedals = 0;
Angles position;
Angles prev_pos;
Angles accel_ang;
Angles zero_ang; //no more memory for pointer declaration?
    
void rx_interrupt() {
    
    char incomingData;
    
    // Note: you need to actually read from the serial to clear the RX interrupt
    incomingData = (BLUETOOTH_ON) ? bluetooth.getc() : usb_pc.getc();
   
        if (data_flag == 1) {
            pedalsData[data_index] = incomingData;
            data_index++;
        }
        else if (data_flag == 2) {
            steerData[data_index] = incomingData;
            data_index++;
        }

        if (incomingData == 's') {data_flag = 1;}
        else if (incomingData == ',') {
            data_flag = 2;
            pedalsData[data_index] = '\0'; //store a null-end at the array
            pedals = atoi(pedalsData);
            data_index = 0; //reset the index to 0
        }
        else if (incomingData == 'e') {
            data_flag = 0;
            steerData[data_index] = '\0'; //store a null-end at the array
            steer = atoi(steerData);
            data_index = 0; //reset the index to 0
        }
}

//***********************************************************************//
//                       User Defined State Machine                      //
//***********************************************************************//
//Enumeration of States.
enum SM1_States {SM1_INIT=1, SM1_CALIBRATE, SM1_WAIT, SM1_MOT_CTR, SM1_STOP};
enum SM2_State {SM2_INIT=1, SM2_READ};

void isr_timer()
{
    timer_flag = 1;
}

void init_tasks(Task *tasks[]) 
{
    // Period for the tasks------------------------------------------------------------
    unsigned short int SMTick1_calc = SM1_TIME; //10 ms
    unsigned short int SMTick2_calc = SM2_TIME; //100 ms
    
    //Greatest common divisor for all tasks or smallest time unit for tasks.
    unsigned long int GCD = CTR_TIMER_MSEC;
    
    
    //Recalculate GCD periods for scheduler---------------------------------------------
    unsigned long int SMTick1_period = SMTick1_calc/GCD;
    unsigned long int SMTick2_period = SMTick2_calc/GCD;
    
    
    // Task 1
    tasks[0]->state = SM1_INIT; //Task initial state.
    tasks[0]->period = SMTick1_period; //Task Period.
    tasks[0]->elapsedTime = SMTick1_period; //Task current elapsed time.
    tasks[0]->TickFct = &SMTick1; //Function pointer for the tick.
    
    // Task 2
    tasks[1]->state = SM2_INIT; //Task initial state.
    tasks[1]->period = SMTick2_period; //Task Period.
    tasks[1]->elapsedTime = SMTick2_period; //Task current elapsed time.
    tasks[1]->TickFct = &SMTick2; //Function pointer for the tick.
    
}

void init_func(void) 
{
   for (int i=0; i<2000; i=i+250) {
      speaker.period(1.0/float(i));
      speaker.write(0.5);
      wait(.05);
   }
    
   speaker = 0.0;
    
    wait(3.0);
    
    if(!BLUETOOTH_ON) {usb_pc.baud(230400); bluetooth.baud(115200); usb_pc.attach(&rx_interrupt);} 
    else {bluetooth.baud(230400); usb_pc.baud(115200); bluetooth.attach(&rx_interrupt);}
    
   imu6.init_all(CTR_TIMER_MSEC);    
   
                 
}
//
//void reset_zero() {
//    
//    Angles zero_reset;
//    Angles actual;
//    int num_itr = 200;
//    
//    mot.set_fullbrake(RIGHT);
//    mot.set_fullbrake(LEFT);
//        
//    //takes num_itr*CTR_TIMER_MSEC time to zero     
//    for(int i = 0; i<num_itr; i++) {
//        runtime.reset();
//        imu6.read_accel_angles();
//        actual.pitch = imu6.pitch_a;
//        actual.roll = imu6.roll_a;
//        mot.move_to(&actual, &zero_reset);
//        while(runtime.read_ms() < CTR_TIMER_MSEC) {wait_us(1);}
//    }
//}



void LED_On(char bitLEDs) {
    
    led1 = (0x01 & bitLEDs);    
    led2 = (0x02 & bitLEDs);  
    led3 = (0x04 & bitLEDs);  
    led4 = (0x08 & bitLEDs);  
    
}

void task_scheduler()
{

//    int temp3 = 0;
    int i = 0;
    isr_task.attach(&isr_timer, 0.001*CTR_TIMER_MSEC); // interval (0.01 seconds)
    
    //Declare an array of tasks-----------------------------------------------------------
    static Task task1,task2;
    Task *tasks[] = { &task1,&task2};
    const unsigned short numTasks = sizeof(tasks)/sizeof(Task*);
    
    init_tasks(tasks); 
    init_func();

//    bluetooth.printf("START");
    
 //   runtime.start();
//    reset_zero();
//    runtime.stop();

   for (int i=2000; i<3000; i=i+125) {
      speaker.period(1.0/float(i));
      speaker.write(0.5);
      wait(.05);
   }
   
   speaker = 0.0;
    
//    if(DEBUG_T) runtime.start();
    
    while(1) {
//        if(DEBUG_T) runtime.reset();
        // Scheduler code
        for ( i = 0; i < numTasks; i++ ) {
            // Task is ready to tick
            if (tasks[i]->elapsedTime == tasks[i]->period) {
                // Setting next state for task
                tasks[i]->state = tasks[i]->TickFct(tasks[i]->state);
                // Reset the elapsed time for next tick.
                tasks[i]->elapsedTime = 0;
            }
            tasks[i]->elapsedTime += 1;
        }
//        if(DEBUG_T) temp3 = runtime.read_us();
        while(!timer_flag){}; //wait here until ISR is ready
//        if(DEBUG_T) {(BLUETOOTH_ON) ? bluetooth.printf("t: %d\r\n",temp3) : usb_pc.printf("t: %d\r\n",temp3);}

        timer_flag = 0;
    }    
    
}

//***********************************************************************//
//                      State Machine:            //
//***********************************************************************//
int SMTick1 (int state) {
       
    static int PositionZero = 0;
    static float sum_calibrateP = 0;
    static float sum_calibrateR = 0;
    static int cnt_calibrate = 0;
    
    imu6.read_accel_angles();
    
    accel_ang.pitch = imu6.pitch_a;
    accel_ang.roll  = imu6.roll_a;
    
    mot.mod_PID(accel_ang.roll);
    
    ((accel_ang.pitch >= MIN_ZERO && accel_ang.pitch <= MAX_ZERO) && 
        (accel_ang.roll >= MIN_ZERO && accel_ang.roll <= MAX_ZERO)) 
            ? (PositionZero = 1) : (PositionZero = 0);
//    (PositionZero) ? (led3 = 1) : (led2 = 1);  

//    PositionZero = 1;
    
    switch(state) {
        case SM1_INIT: 
            mot.set_fullbrake(RIGHT);
            mot.set_fullbrake(LEFT);
            state = SM1_CALIBRATE;
            cnt_calibrate = 0;
            break;

        case SM1_CALIBRATE:
            
            if (cnt_calibrate < (1000/SM1_TIME)) { //1 sec calibration
                sum_calibrateP += accel_ang.pitch;
                sum_calibrateR += accel_ang.roll;
                state = SM1_CALIBRATE;
            }
            else {
                sum_calibrateP /= (1000/SM1_TIME);
                sum_calibrateR /= (1000/SM1_TIME);
                state = SM1_WAIT;
            }
            cnt_calibrate++;      
            break;
            
        case SM1_WAIT:
            start_flag = 0;
            move_flag = 0;
            zero_ang.pitch = INIT_PITCH;
            zero_ang.roll = INIT_ROLL;

            if (sw_BL || sw_FL || sw_BR || sw_FR || BtnStop) {
                mot.stop_hard(); 
                state = SM1_STOP; 
            }
            else if (!(PositionZero && BtnStart)) {
                if (time_count > (SM2_TIME/SM1_TIME)) {time_count = SM2_TIME/SM1_TIME;}
                zero_ang.pitch = ((float)(0.0 - sum_calibrateP)) / ((float)(SM2_TIME/SM1_TIME)) * ((float)time_count) + sum_calibrateP;
                zero_ang.roll  = ((float)(0.0 - sum_calibrateR)) / ((float)(SM2_TIME/SM1_TIME)) * ((float)time_count) + sum_calibrateR;            
                mot.position_control(&zero_ang, &accel_ang);
                time_count++;
                state = SM1_WAIT;   
            } 
            else {
                prev_pos.pitch = 0.0;
                prev_pos.roll = 0.0;
                start_flag = 1;
                position.roll = TEST_ROLL;
                position.pitch = TEST_PITCH;
                time_count = 1;
                state = SM1_MOT_CTR;  
            }  

            LED_On(0x01);
            break;
            
        case SM1_MOT_CTR: 
            
            if (time_count > (SM2_TIME/SM1_TIME)) {time_count = SM2_TIME/SM1_TIME;}
            if (move_flag) {
                zero_ang.pitch = ((float)(position.pitch - prev_pos.pitch)) / ((float)(SM2_TIME/SM1_TIME)) * ((float)time_count) + prev_pos.pitch;
                zero_ang.roll  = ((float)(position.roll - prev_pos.roll)) / ((float)(SM2_TIME/SM1_TIME)) * ((float)time_count) + prev_pos.roll;
            }
            else {
                zero_ang.pitch = 0;
                zero_ang.roll = 0;
            }
                
            if (!BtnStop && !(sw_BL || sw_FL || sw_BR || sw_FR)) {                
//                mot.velocity_control(&position, &accel_ang);
                mot.position_control(&zero_ang, &accel_ang);
                time_count++;
                state = SM1_MOT_CTR;
            }
            else if(BtnStart && BtnStop) {
                state = SM1_WAIT;
            }
            else {
                mot.stop_hard(); 
                state = SM1_STOP;
            }
            LED_On(0x02);
            break;
        
        case SM1_STOP: //TODO: Light up a Warning LED here
            if (BtnReset && !BtnStop) {
                state = SM1_INIT; 
            }
            else {
                mot.stop_hard(); 
                state = SM1_STOP;   
            }
            LED_On(0x04);
            break;
            
        default:
            state = SM1_INIT;
            break;
    }
    
            (!BLUETOOTH_ON) ?
                bluetooth.printf("%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\r\n", zero_ang.pitch,zero_ang.roll,accel_ang.pitch,accel_ang.roll,imu6.actual_accel.y)
                : usb_pc.printf("%4.3f,%4.3f,%4.3f,%4.3f,%4.3f\r\n", zero_ang.pitch,zero_ang.roll,accel_ang.pitch,accel_ang.roll,imu6.actual_accel.y);
    
    return state;
}

int SMTick2 (int state) {
    static unsigned int cnt = 0;
    int iter = 1;
    
    switch(state) {
        
        case SM2_INIT: 
            position.roll = INIT_ROLL;
            position.pitch = INIT_PITCH;
            prev_pos.pitch = INIT_PITCH;
            prev_pos.roll = INIT_ROLL;
            cnt = 1;
            steer = (MAX_STEER+MIN_STEER)/2;
            pedals = (MAX_PEDALS + MIN_PEDALS);
//            count = 1;
            state = (!start_flag) ? (SM2_INIT) : (SM2_READ);
            break;

        case SM2_READ:
              time_count = 1;
              move_flag = 1;
//            position.roll = ((float) steer)*(MAX_ROLL-MIN_ROLL)/(MAX_STEER-MIN_STEER) + (MIN_ROLL);
//            position.pitch = ((float) pedals)*(MAX_PITCH-MIN_PITCH)/(MAX_PEDALS - MIN_PEDALS);
            //prev_pos.pitch = position.pitch;
//            prev_pos.roll = position.roll;
//            
            //prev_pos.pitch = 0.0;
//            prev_pos.roll = 0.0;
//          
            if (cnt <= iter) {  
                prev_pos.pitch = position.pitch;
                prev_pos.roll = position.roll;
                position.roll = TEST_ROLL;
                position.pitch = TEST_PITCH;
            }
            else if (cnt > iter && cnt <= 2*iter) {
                prev_pos.pitch = position.pitch;
                prev_pos.roll = position.roll;
                position.roll = TEST_ROLL;
                position.pitch = -TEST_PITCH;
            }
            else if (cnt > 2*iter && cnt <= 3*iter) {
                prev_pos.pitch = position.pitch;
                prev_pos.roll = position.roll;
                position.roll = 5;
                position.pitch = 0;
            }
            else if (cnt > 3*iter && cnt <= 4*iter) {
                prev_pos.pitch = position.pitch;
                prev_pos.roll = position.roll;
                position.roll = -5;
                position.pitch = 0;
            }
            else if (cnt > 4*iter) {
                prev_pos.pitch = position.pitch;
                prev_pos.roll = position.roll;
                position.roll = 0;
                position.pitch = 0;
                cnt = 0;
            }
            cnt++;
            state = SM2_READ;
            break;
            
            
        default:
            state = SM2_INIT;
            break;            
    }
    
    return state;   
}