/* This code is copied from 
 * "https://github.com/pololu/l3g-arduino/blob/master/L3G/L3G.cpp"
 * used for arduino and modified to use for mbed
 */
#include "L3G.h"
#include <math.h>
extern I2C i2c;

//The mbed API uses 8 bit addresses, so make sure to take that
//7 bit address and left shift it by 1 before passing it.

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
// pololu l3gd20 is 1101011b by default
//L3GD20H (gyro)    1101011b    1101010b

//left shift 1 for mbed 
#define D20_SA0_HIGH_ADDRESS      (0x6B<<1) // also applies to D20H
#define D20_SA0_LOW_ADDRESS       (0x6A<<1) // also applies to D20H

#define TEST_REG_ERROR  -1

#define D20H_WHO_ID     0xD7
#define D20_WHO_ID      0xD4

// Constructors ////////////////////////////////////////////////////////////////

L3G::L3G(void)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in read() since the last call to timeoutOccurred()?
bool L3G::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void L3G::setTimeout(unsigned int timeout)
{
  io_timeout = timeout;
}

unsigned int L3G::getTimeout()
{
  return io_timeout;
}

bool L3G::init(deviceType device, sa0State sa0)
{
  int id;

  // perform auto-detection unless device type and SA0 state were both specified
  if (device == device_auto || sa0 == sa0_auto)
  {
    // check for L3GD20H, D20 if device is unidentified or was specified to be one of these types
    if (device == device_auto || device == device_D20H || device == device_D20)
    {
      // check SA0 high address unless SA0 was specified to be low
      id = testReg( D20_SA0_HIGH_ADDRESS, WHO_AM_I);
      if (sa0 != sa0_low && (id) != TEST_REG_ERROR)
      {
        // device responds to address 1101011; it's a D20H or D20 with SA0 high     
        sa0 = sa0_high;
        if (device == device_auto)
        {
          // use ID from WHO_AM_I register to determine device type
          device = (id == D20H_WHO_ID) ? device_D20H : device_D20;
        }
      }
      
      // check SA0 low address unless SA0 was specified to be high
      id = testReg((char) D20_SA0_LOW_ADDRESS, (regAddr) WHO_AM_I);
      if (sa0 != sa0_high && (id != TEST_REG_ERROR)) 
      {
        // device responds to address 1101010; it's a D20H or D20 with SA0 low      
        sa0 = sa0_low;
        if (device == device_auto)
        {
          // use ID from WHO_AM_I register to determine device type
          device = (id == D20H_WHO_ID) ? device_D20H : device_D20;
        }
      }
    }
    
    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }
  
  _device = device;

  // set device address
  switch (device)
  {
    case device_D20H: 
      address = (sa0 == sa0_high) ? D20_SA0_HIGH_ADDRESS : D20_SA0_LOW_ADDRESS;
      break;

    case device_D20:
      address = (sa0 == sa0_high) ? D20_SA0_HIGH_ADDRESS : D20_SA0_LOW_ADDRESS;
      break;
  }
  
  return true;
}

/*
Enables the L3G's gyro. Also:
- Sets gyro full scale (gain) to default power-on value of +/- 250 dps
  (specified as +/- 245 dps for L3GD20H).
- Selects 200 Hz ODR (output data rate). (Exact rate is specified as 189.4 Hz
  for L3GD20H and 190 Hz for L3GD20.)
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void L3G::enableDefault(void)
{
  if (_device == device_D20H)
  {
    // 0x00 = 0b00000000
    // Low_ODR = 0 (low speed ODR disabled)
    writeReg(LOW_ODR, 0x00);
  }
  
  // 0x00 = 0b00000000
  // FS = 00 (+/- 250 dps full scale) (L3GD20H is 245)
  // FS = 01 (+/- 500 dps full scale)
  // FS = 10 (+/- 2000 dps full scale)
  writeReg(CTRL_REG4, (0x00<<4));
  
  // 0x6F = 0b01101111
  // DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
  writeReg(CTRL_REG1, 0x6F);
  
  //Enable High Pass Filter
//  writeReg(CTRL_REG5, 0x10);
  
  //Set High Pass Filter
  //0x20: Normal Mode
  //0x07: 0.09 (cutoff Hz @ 250dps)
//  writeReg(CTRL_REG2,0x27);
}

// Writes a gyro register
void L3G::writeReg(char reg, char value)
{
    char write_buff[2]={reg, value};
    i2c.write(address,write_buff,2);
    
}

// Reads a gyro register
char L3G::readReg(char reg)
{
  char value[1]={0};
  char write_reg[1]={reg};
 // Wire.beginTransmission(address);
//  Wire.write(reg);
//  last_status = Wire.endTransmission();
//  Wire.requestFrom(address, (byte)1);
//  value = Wire.read();
//  Wire.endTransmission();

    i2c.write(address,write_reg,1);
    i2c.read(address,value,1);

  return value[0];
}

// Reads the 3 gyro channels and stores them in vector g
void L3G::read()
{
//  Wire.beginTransmission(address);
//  // assert the MSB of the address to get the gyro
//  // to do slave-transmit subaddress updating.
//  Wire.write(OUT_X_L | (1 << 7));
//  Wire.endTransmission();
//  Wire.requestFrom(address, (byte)6);
  
  // assert the MSB of the address to get the gyro
  // to do slave-transmit subaddress updating.
  char EnableRead_gyro[1] = {OUT_X_L|(1 << 7)};
  i2c.write(address,EnableRead_gyro,1);
 // unsigned int millis_start = millis();
//  while (Wire.available() < 6)
//  {
//    if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
//    {
//      did_timeout = true;
//      return;
//    }
//  }

//  char xlg = Wire.read();
//  char xhg = Wire.read();
//  char ylg = Wire.read();
//  char yhg = Wire.read();
//  char zlg = Wire.read();
//  char zhg = Wire.read();

//    char gyro_data[6]={0};  //xlg, xhg, ylg, yhg, zlg, zhg;
    char xlg, xhg, ylg, yhg, zlg, zhg;
   
    //
//    i2c.read(address,xlg,1);
//    i2c.read(address,xhg,1);
//    i2c.read(address,ylg,1);
//    i2c.read(address,yhg,1);
//    i2c.read(address,zlg,1);
//    i2c.read(address,zhg,1);

    xlg = readReg(OUT_X_L);   
    xhg = readReg(OUT_X_H);   
    ylg = readReg(OUT_Y_L);   
    yhg = readReg(OUT_Y_H);   
    zlg = readReg(OUT_Z_L);   
    zhg = readReg(OUT_Z_H);   
    
    // combine high and low bytes
    g.x = (int16_t)(xhg << 8 | xlg);
    g.y = (int16_t)(yhg << 8 | ylg);
    g.z = (int16_t)(zhg << 8 | zlg);
}

void L3G::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a,a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

int L3G::testReg(int address, regAddr reg)
{
    int ret_flag = 0;
    char temp[1] = {0};
    char write_temp[1]={reg};

//  Wire.beginTransmission(address);
//  Wire.write((byte)reg);

  ret_flag = i2c.write(address, write_temp, 1);
  if (ret_flag != 0)
  {
    return TEST_REG_ERROR;
  }

  ret_flag = i2c.read(address, temp, 1);
  if (ret_flag != 0)
  {
    return TEST_REG_ERROR;
  }
  else {
      return ((int) temp[0]);
  }
}

