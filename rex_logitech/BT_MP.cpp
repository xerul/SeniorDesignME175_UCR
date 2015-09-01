#include "BT_MP.h"
#include <string.h>

#define DEVICE_PORT             "/dev/ttyACM0"     
#define BAUDRATE               115200


BT_MP::BT_MP(void)
{
	//put constructor code here...
	pedals = 0;
	brakes = 0;
	throttle = 0;
	steering = 8149;
}

BT_MP::~BT_MP(void)
{
	//put destructor code here...
}

void BT_MP::init_serial()
{

	//sets up serial port
	mRet = mSerial.Open(DEVICE_PORT,BAUDRATE);                              // Open serial link at 115200 bauds
	
	while (mRet!=1) {                                                          // If an error occured...
		printf("\nunable to connect to serial ... trying again!\n\n");
		mRet = mSerial.Open(DEVICE_PORT,BAUDRATE);                              // Open serial link at 115200 bauds
	}

	printf("Succesfully connected!\r\n");

}

void BT_MP::write_serial(int a, int b)
{

	char intStr[128];
	std::string intString;


	snprintf(intStr, sizeof(intStr), "%d", a);
        intString = std::string(intStr);

	mRet = mSerial.WriteString("s");
	if (mRet!=1) { printf ("Error while writing s\n");} 
	
	mRet = mSerial.WriteString(intStr);      
	if (mRet!=1) { printf ("Error while writing string data\n");} 

	mRet = mSerial.WriteString(",");
	if (mRet!=1) { printf ("Error while writing ,\n");} 
	
	snprintf(intStr, sizeof(intStr), "%d", b);
	intString = std::string(intStr);

	mRet = mSerial.WriteString(intStr);                              // Send the command on the serial port
	if (mRet!=1) { printf ("Error while writing string data\n");} 

	mRet = mSerial.WriteString("e");
	if (mRet!=1) { printf ("Error while writing e\n");} 
}
	
