
/*
 *  Gripper_Communicate.h
 *  
 *  Created on: 01 Apr 2024
 *  Author: M.Teerachot
 *  Tested with STM32G431CB
 */

/*
 *  Note:
 *  (*) -> the most important functions
 */

#ifndef GRIPPER_COMMUNICATE_H
#define GRIPPER_COMMUNICATE_H

/* (1) config pin */ 
#define SERIAL2_RXPIN       PB4
#define SERIAL2_TXPIN       PB3

/* (2) create serial object */ 
HardwareSerial Serial2(SERIAL2_RXPIN, SERIAL2_TXPIN); 

/* (3) variable */
// 3.1 printing 
//#define GRIPPER_PACKET_PRINT
//#define PICKUP_PRINT
//#define BASKET_PRINT
// 3.2 constant
#define BASKET_OFF      0
#define BASKET_ON       1
#define GRPC_TIMEOUT    -1
#define GRPC_NOT_READY  -2
#define GRPC_INCORRECT  -3
// refer: Gripper.h -> 2.3 error identify
#define GRP_REACHED     1
#define GRP_TIMEOUT     -1   
#define GRP_ALARM       -2
#define GRP_ZERO_MOVE   -3
#define GRP_NOT_READY   -11
#define GRP_HOMED       -12
#define GRP_HOMING      -13
#define GRP_SUCCEED     -14
#define GRP_MISTAKE     -15
#define GRP_DEPLETED    -16
// 3.3 dynamic
int randNumber, prevRandNum = 0;

/* (4) communication functions */
//  (*) base function for send a packet to gripper unit
int sendGripperPacket(int cmd, int timeout=1000)
{
  Serial2.flush();
  int ret = GRPC_NOT_READY;
  
  // 1.1 packet generate 
  while(1)
  {
    randNumber = random(1, 99);
    if (randNumber != prevRandNum) 
    {
      prevRandNum = randNumber;
      break;       
    }
  }
  // 1.2 send packet
  Serial2.write('*');
  Serial2.println(cmd);
  Serial2.println(randNumber);
  Serial2.println(cmd + randNumber);
  
  // 2. waiting the return packet
  unsigned long timer = millis();
  while(1)
  {
    if (millis()-timer >= timeout)
    {
      ret = GRPC_TIMEOUT;
      break;
    }
    if (Serial2.find('&'))
    {
      int cmdRet = Serial2.parseInt();
      int numRet = Serial2.parseInt();
      int sumRet = Serial2.parseInt();
#ifdef GRIPPER_PACKET_PRINT
      Serial.print("cmd: ");     Serial.print(cmdRet);
      Serial.print("\tnum: ");   Serial.print(numRet);
      Serial.print("\tsum: ");   Serial.println(sumRet);
#endif
      // correct packet
      if (cmdRet + numRet == sumRet) 
        ret = numRet;
      else 
        ret = GRPC_INCORRECT;
      break;
    }
  }
  return ret;
}

void sendGripperRetPacket(Stream *serial, int cmd, int randNum)
{
  serial->write('&');
  serial->println(cmd);
  serial->println(randNum);
  serial->println(cmd+randNum);
}    

void ping()
{
  unsigned long startTime = millis();
  int ret = sendGripperPacket(9, 2000);
  unsigned long period = millis()-startTime;
  Serial.print("Command: ");
  Serial.print(ret);
  Serial.print("\tPeriod: ");
  Serial.println(period);
}

/* (5) gripper function */
int gripperHome(int timeout=2000)
{
  return sendGripperPacket(4, timeout); 
}

int basketHome(int timeout=2000)
{
  return sendGripperPacket(5, timeout); 
}

int basketMove(int cmd, bool preEnable, bool postDisable, int timeout=4000)
{
  int ret = GRP_NOT_READY;
  
  // 1. motor enable
  if (preEnable)
  { 
    ret = sendGripperPacket(3, 1500); 
#ifdef BASKET_PRINT
    Serial.print("Basket: ");
    Serial.print(ret);
    Serial.println(" (enable)"); 
#endif
  }
  
  // 2. start process
  switch (cmd)
  {
    case BASKET_ON:
      ret = sendGripperPacket(7, timeout);
      break;
      
    case BASKET_OFF:
      ret = sendGripperPacket(8, timeout);
      break;  
  }
#ifdef BASKET_PRINT
  Serial.print("Basket: ");
  Serial.print(ret);
  Serial.println(" (move)"); 
#endif

  // 3. motor disable
  if (postDisable)
  { 
    ret = sendGripperPacket(2, 1500); 
#ifdef BASKET_PRINT
    Serial.print("Basket: ");
    Serial.print(ret);
    Serial.println(" (disable)"); 
#endif
  }
  return ret;
}

bool fullGripperHome(Stream *serial, String headPacket, String endPacket, int timeout)
{
  timeout = abs(timeout);
  unsigned long timer;
  int ret;
  bool gripperGotHome = false;
  bool basketGotHome = false;
  
  // 1.1 home basket
  ret = basketHome(timeout);
  if (ret == GRP_HOMED)   basketGotHome = true;
  else                    basketGotHome = false;
  
  // 1.2 home gripper
  ret = gripperHome(timeout);
  if (ret == GRP_HOMED)   gripperGotHome = true;
  else                    gripperGotHome = false;
  
  if (gripperGotHome && basketGotHome)  return 1;
  else                                  
  {
    serial->println(headPacket + 'T' + endPacket);
    return 0;
  }
}

bool getTray(int timeout=1500)
{
  bool ret = 0;
  bool retPacket = sendGripperPacket(10, timeout);
  if (retPacket == 0 || retPacket == 1) ret = retPacket;
  return ret;
}

#endif
