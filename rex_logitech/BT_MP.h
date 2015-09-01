
#ifndef BT_MP_H
#define BT_MP_H

#include <cstdio>
#include "serialib.h"
#include <stdlib.h>
#include "BT_MP.h"

class BT_MP
{
 public:

	int pedals;
	int brakes;
	int throttle;
	int steering;

	BT_MP();
	~BT_MP();
	void init_serial();
	void write_serial(int a, int b);

 private: 
	serialib mSerial;                                                      
	int mRet;                                                              
	char mBuffer[128];
	

};

#endif
