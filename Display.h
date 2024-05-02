
/*
 *  Display.h
 *  
 *  Created on: 12 Mar 2024
 *  Author: M.Teerachot
 *  Tested with STM32G431CB
 */
 
#ifndef DISPLAY_H
#define DISPLAY_H

/* (1) constant variables */
// 1. serial communication
#define SERIAL3_RXPIN_CH1   PB11
#define SERIAL3_TXPIN_CH1   PB10
#define SERIAL3_RXPIN_CH2   PC11
#define SERIAL3_TXPIN_CH2   PC10
#define SERIAL3_CH1     31
#define SERIAL3_CH2     32
// (2) constant variables
// 1. packet
#define MOD_HEADER      '#'
#define ID_MASTER       0x00
#define ID_BROADCAST    0x3F
// 2. display
#define CL_RED          1
#define CL_GREEN        2
#define CL_BLUE         3
#define CL_YELLOW       4

const uint8_t channelToId[8][10]
{
  {0,   0,   0,   0,   0,   0,   0,   0,   0,   0},
  {0,   1,   2,   3,   4,   5,   6,   7,   8,   0},
  {0,   9,  10,  11,  12,  13,  14,  15,  16,   0},
  {0,  17,  18,  19,  20,  21,  22,  23,  24,   0},
  {0,  25,  26,  27,  28,  29,  30,  31,  32,   0},
  {0,  33,  34,  35,  36,  37,  38,  39,  40,   0},
  {0,  41,  42,  43,  44,  45,  46,  47,  48,   0},
  {0,  49,  50,  51,  52,  53,  54,  55,  56,   0}
};

/* (3) create serial object */ 
HardwareSerial Serial3(SERIAL3_RXPIN_CH1, SERIAL3_TXPIN_CH1);

void selectSerial3(int ch, int baud, int timeout)
{
  if (ch == SERIAL3_CH1)
  {
    pinMode(SERIAL3_RXPIN_CH2, INPUT);
    pinMode(SERIAL3_TXPIN_CH2, INPUT);
    Serial3.setRx(SERIAL3_RXPIN_CH1);
    Serial3.setTx(SERIAL3_TXPIN_CH1);
  }
  else if (ch == SERIAL3_CH2)
  {
    pinMode(SERIAL3_RXPIN_CH1, INPUT);
    pinMode(SERIAL3_TXPIN_CH1, INPUT);
    Serial3.setRx(SERIAL3_RXPIN_CH2);
    Serial3.setTx(SERIAL3_TXPIN_CH2);
  }
  Serial3.begin(baud);
  Serial3.setTimeout(timeout);
}

// (*) base function for send a packet to display unit
void sendModulePacket(int id, int cmd, int data)
{
  Serial3.write(MOD_HEADER);
  Serial3.println(id);
  Serial3.println(cmd);
  Serial3.println(data);
  Serial3.println(id + cmd + data); 
}

// (4) set color on display
void setDistColor(int id, int cl, int prd)
{
  // 1. set pwm  
  sendModulePacket(id, 19, 127);
  // 2. set period
  sendModulePacket(id, 20, prd);
  // 3. enable blink
  sendModulePacket(id, 20+cl, 0);
}

void resetDistColor(int id)
{
  // 1. disable blink
  sendModulePacket(id, 29, 0);
}

// (5) set number on display
void setDistNumber(int id, int num)
{
  sendModulePacket(id, 31, num);
}

void resetDistNumber(int id)
{
  sendModulePacket(id, 32, 0);
}

// (6) test function
// upstairs:    1-24
// downstairs:  25-56 

void testAllDisplay(int cl)
{
  for (int row=1; row<=7; row++)
  {
    for (int col=1; col<=8; col++)
    {
      int id = (row-1) * 8 + col;
      setDistColor(id, cl, 20);
      delay(5);
      setDistNumber(id, id);
      delay(5);
    }
    delay(2000);
    resetDistColor(ID_BROADCAST);
    delay(5);
    resetDistNumber(ID_BROADCAST);
    delay(5);
  }
}

// (*for test) base function for send and receive a packet from hub unit
int sendHubPacket(int id, int cmd, int data, int timeout=1500)
{
  // 1. send packet
  Serial3.write(MOD_HEADER);
  Serial3.println(id);
  Serial3.println(cmd);
  Serial3.println(data);
  Serial3.println(id + cmd + data); 

  unsigned long timer = millis();
  while(1)
  {
    // 2.1 timeout
    if (millis()-timer >= timeout)
      return -1;

    // 2.2 get packet
    if (Serial3.find('&'))
    {
      int idRet   = Serial3.parseInt();
      int cmdRet  = Serial3.parseInt();
      int dataRet = Serial3.parseInt();
      int sumRet  = Serial3.parseInt();
      int sum = idRet + cmdRet + dataRet;
     
      // 2.3 packet identify
      if (sum != sumRet)  
        return -2;
  
      // 2.4 module identify
      if (idRet == id)
        return dataRet;
    }
  }
  return -3;
}

int getShelf()
{
  int maxCnt = 3;
  int state = -1;
  for (int s=0; s<maxCnt; s++)
  { 
    state = sendHubPacket(70, 2, 0, 1500);
    if (state != -1)  break;
  }
  return state;
}


#endif
