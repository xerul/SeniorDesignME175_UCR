#ifndef TASK_CONTROL_H
#define TASK_CONTROL_H

#include <stdlib.h>
#include "mbed_init.h"
#include "IMU_6DOF.h"
#include "motor_control.h"

//***********************************************************************//
//                        Task Scheduler Data Structure                  //
//***********************************************************************//
typedef struct _task { 
/*Tasks should have members that include: state, period,
a measurement of elapsed time, and a function pointer.*/

    signed char state; //Task's Current State
    unsigned long int period; //Task Period
    unsigned long int elapsedTime; //Time elapsed since last task tick
    int (*TickFct) (int); //Task tick function
    
} Task;

int SMTick1 (int state);
int SMTick2 (int state);

void isr_timer();
void init_tasks(Task *tasks[]); 
void init_func(); 
void task_scheduler();

void LED_On(char bitLEDs);

#endif
