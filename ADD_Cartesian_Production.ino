

/*
    ADD_Cartesian_Production.ino

    Created on: 24 Apr 2024
    Author: M.Teerachot
    Tested with STM32G431CB
*/
#include "System.h"
#include "Cartesian.h"
#include "Display.h"
#include "Gripper_Communicate.h"
#include "Interface.h"

#define sleepTimeSet   5000  // 45 second
unsigned long sleepTimer;
bool sleepState = false;

int gripperPickup(int row=1, int col=1, int qtSet=1, int mistakeSet=2, int timeout=4000)
{
  // 0. initial variables 
  int qtNow = 0;
  int mistakeCnt = mistakeSet;
  
  // 1. motor enable
  int ret = sendGripperPacket(1, 1500); 
#ifdef PICKUP_PRINT 
  Serial.print("Gripper: ");
  Serial.print(ret);
  Serial.println(" (enable)"); 
#endif
  if (ret != GRP_SUCCEED)   return GRP_NOT_READY;   
  
  // 2. pick
  while (mistakeCnt)
  {
    ret = sendGripperPacket(6, timeout);
#ifdef PICKUP_PRINT 
  Serial.print("Gripper: ");
  Serial.print(ret);
  Serial.println(" (pick)"); 
#endif
    // 2.1 continue...
    if (ret == GRP_SUCCEED) 
    {
      qtNow++;
      mistakeCnt = mistakeSet;
      setDistNumber(channelToId[row][col], qtSet-qtNow);
    }
    
    // 2.2 timeout
    else  if (ret == GRP_MISTAKE || ret == GRP_TIMEOUT) 
      mistakeCnt--;   

    // 2.3 deplete
    else if (ret == GRP_DEPLETED)
       mistakeCnt = 0;  
    
    // 2.4 complete
    if (qtNow == qtSet) 
      break; 
  }

  // 3. motor disable
  ret = sendGripperPacket(0, 1500);  // disable  
#ifdef PICKUP_PRINT 
  Serial.print("Gripper: ");
  Serial.print(ret);
  Serial.println(" (disable)"); 
#endif
  return qtNow;
}

void interface(Stream *serial)
{
//-> 0. example packet: B0916D0004S1D1001R1,
//                      B0926D0003S1D1001R1,
//                      B0958D0002S1D1234R1,
  
//-> 1. get packet
  if (serial->find('B'))
  {
    packet    = serial->readStringUntil(',');
    cabinet   = packet.substring(0, 2);
    row       = packet.substring(2, 3);
    column    = packet.substring(3, 4);
    dispQt    = packet.substring(5, 9);
    cmdState  = packet.substring(10, 11);
    device    = packet.substring(12, 16);
    tray      = packet.substring(17, 18);
    
//-> 2. convert and prepare variables
    cabinetInt  = StringToInt(cabinet, 2);
    rowInt      = StringToInt(row, 1);
    columnInt   = StringToInt(column, 1);
    dispQtInt   = StringToInt(dispQt, 4);
    cmdStateInt = StringToInt(cmdState, 1);
    deviceInt   = StringToInt(device, 4);    
    trayInt     = StringToInt(tray, 1); 
    
//#define SHOW_PARAM
#ifdef SHOW_PARAM
//    Serial.print(F("0. Packet\t\t"));         Serial.println(packet);
//    Serial.print(F("1. Cabinet\t\t"));        Serial.println(cabinet);    
//    Serial.print(F("2. Row\t\t\t"));          Serial.println(row); 
//    Serial.print(F("3. Column\t\t"));         Serial.println(column);    
//    Serial.print(F("4. Dispensed Qt.\t"));    Serial.println(dispQt);    
//    Serial.print(F("5. Command state\t"));    Serial.println(cmdState);    
//    Serial.print(F("6. Device\t\t"));         Serial.println(device);      
//    Serial.print(F("7. Tray\t\t\t"));         Serial.println(tray);      

    Serial.print(F("1. Cabinet\t\t"));        Serial.println(cabinetInt);    
    Serial.print(F("2. Row\t\t\t"));          Serial.println(rowInt); 
    Serial.print(F("3. Column\t\t"));         Serial.println(columnInt);    
    Serial.print(F("4. Dispensed Qt.\t"));    Serial.println(dispQtInt);    
    Serial.print(F("5. Command state\t"));    Serial.println(cmdStateInt);    
    Serial.print(F("6. Device\t\t"));         Serial.println(deviceInt);    
    Serial.print(F("7. Tray\t\t\t"));         Serial.println(trayInt);      
#endif

    String retHeadPacket = 'B' + cabinet + row + column + 'D' + dispQt + "S0D" + device;
    String retEndPacket = "0000R0";
    
//-> 3. running procedure

//-> 3.1 griper and basket home
    bool retHomeGripper = fullGripperHome(serial, retHeadPacket, retEndPacket, 1500);
    if (!retHomeGripper) return;

//-> 3.2 vertical and honrizontal home
    bool retHomeHonrizontal = true;
    bool retHomeVertical = true;

//->     
    if (!getTray())
    {
      retHomeHonrizontal = homeHonrizontal(serial, retHeadPacket, retEndPacket, 1600*2, 35000); 
      if (!retHomeHonrizontal) return;
    
      retHomeVertical = homeVertical(serial, retHeadPacket, retEndPacket, 1600, 45000); 
      if (!retHomeVertical) return;

      zeroRalative();
    }

//-> 
    bool retShelf = getShelf();
    
    // safty checklists
    if (retHomeHonrizontal && retHomeVertical && retHomeGripper && retShelf)
      serial->println(retHeadPacket + 'Z' + retEndPacket);
    else 
    {
      serial->println(retHeadPacket + 'T' + retEndPacket);
      return;
    }
    
//-> 3.3 reset and set target display
    resetDistColor(ID_BROADCAST);
    resetDistNumber(ID_BROADCAST);

    setDistNumber(channelToId[rowInt][columnInt], dispQtInt);
    setDistColor(channelToId[rowInt][columnInt], CL_BLUE, 250);

//-> 3.4 go to module
    enable(CARTESIAN); 
    int retTarvel = cartesianTravelChannel(rowInt, columnInt, false, false);  
    if (retTarvel != REACHED)
    {
      serial->println(retHeadPacket + 'E' + retEndPacket);
      return;
    }
    
//-> 3.5 pick up a medicine
    setDistColor(channelToId[rowInt][columnInt], CL_YELLOW, 250);
    int dispQtReal = gripperPickup(rowInt, columnInt, dispQtInt);
    if (dispQtReal == GRP_NOT_READY)
    {
      serial->println(retHeadPacket + 'E' + retEndPacket);
      return;
    }
    
//-> 3.6 update display 
//    setDistNumber(channelToId[rowInt][columnInt], dispQtReal);
    
    // succeed
    if (dispQtReal == dispQtInt)
    {
      setDistColor(channelToId[rowInt][columnInt], CL_GREEN, 250);
      retHeadPacket = 'B' + cabinet + row + column + 'D' + dispQt + "S1D" + device;
    }
    // mistake
    else      
    {                    
      setDistColor(channelToId[rowInt][columnInt], CL_RED, 250);
      retHeadPacket = 'B' + cabinet + row + column + 'D' + dispQt + "S0D" + device;
    }

         if (dispQtReal > 999)   retEndPacket = String(dispQtReal) + tray;        
    else if (dispQtReal > 99)    retEndPacket = "0" + String(dispQtReal) + "R" + tray;
    else if (dispQtReal > 9)     retEndPacket = "00" + String(dispQtReal) + "R" + tray;
    else                         retEndPacket = "000" + String(dispQtReal) + "R" + tray;  
    
//-> 3.7 go to tray
    trayInt = constrain(trayInt, 1, 2);
    switch (trayInt)
    {
      case 1:   // left
        retTarvel = cartesianTravelRelative(150, 850, false, false);   
        break;
      case 2:   // right
        retTarvel = cartesianTravelRelative(0, 850, false, false);   
        break;
    }
    
    if (retTarvel != REACHED)
    {
      serial->println(retHeadPacket + 'E' + retEndPacket);
      return;
    }
    
//-> 3.8 pour medicine
    int retMoveBasket = basketMove(BASKET_ON, true, false);
    if (retMoveBasket != GRP_REACHED)
    {
      serial->println(retHeadPacket + 'E' + retEndPacket);
      return;
    }
    delay(1000);
    
    retMoveBasket = basketMove(BASKET_OFF, false, true);
    // if (retMoveBasket != GRP_REACHED || retMoveBasket != GRP_SUCCEED)   return;

    serial->println(retHeadPacket + 'Z' + retEndPacket);

//-> 3.9 go home
//    retTarvel = cartesianTravelRelative(0, 0, false, false); 
//    disable(CARTESIAN);
//    if (retTarvel != REACHED)   return;
    
//->3.10 reset display
    resetDistNumber(channelToId[rowInt][columnInt]);
    resetDistColor(channelToId[rowInt][columnInt]);

    sleepTimer = millis();
    sleepState = false;
  }
}

void develop(Stream *serial)
{
  if (serial->find('%'))
  {
    int cmd = serial->parseInt();
    Serial.print("Cmd: ");
    Serial.println(cmd);
    
    switch (cmd)
    {
      case 0:
        homeHonrizontal(serial, "1234", "5678", 1600, 20000); // timeout = 20 sec
        break;
      case 1:
        togHorizontal(80500, 20000, 20000, 200000, 3000, 1); 
        break;  
      case 2:
        homeVertical(serial, "8765", "4321", 1600, 25000); // timeout = 25 sec
        break;
      case 3:
        togVertical(60000, 20000, 20000, 200000, 3000, 1);
        break;
      case 4:
        //togSynchronize(62500/5, 20000/4, 20000/4, 50000/4, 3000, 1);
        togSynchronize(12500, 4000, 4000, 120000, 3000, 1);
        break;
      case 5:
        togCartesian(1450, 1133, 3000, 1);
        break;
      case 6:    
        togCartesianRelative(397, 190, 500, true);
        break;
      case 7:
        togCartesianRelative(0, 0);
        break;
      case 8:
        togCartesianTravelChannel(3, 4, 500, 1);   
        break;
      case 9:
        testAllDisplay(CL_RED);        
        break;
      case 10:
        Serial.println(gripperHome());
        break;
      case 11:
        Serial.println(gripperPickup(1, 8, 4));
        break;
      case 12:
        Serial.println(basketHome());
        break;
      case 13:
        Serial.println(fullGripperHome(serial, "8765", "4321", 2000));
        break; 
      case 14: 
        Serial.println(basketMove(BASKET_ON, true, false));
        delay(3000);
        Serial.println(basketMove(BASKET_OFF, false, true));
        break;
      case 15:
        
        break;
        
    }
  }
}

void setup()
{
#ifdef SYSTEM_H
  selectSerial1(SERIAL1_UART);
  pinMode(TOG_UP_PIN, INPUT);
  pinMode(TOG_DOWN_PIN, INPUT);
  pinMode(DETECT24V_PIN, INPUT);
  pinMode(LEDR_ONBOARD_PIN, OUTPUT);
  pinMode(LEDG_ONBOARD_PIN, OUTPUT);
  pinMode(BUZZ_PIN, INPUT); // disable buzzer
#endif

#ifdef CARTESIAN_H
  pinMode(ALMV_PIN, INPUT);
  pinMode(ALMH_PIN, INPUT);
  pinMode(LIMITV_PIN, INPUT);
  pinMode(LIMITH_PIN, INPUT);
  pinMode(BKA_PIN, OUTPUT);
  pinMode(BKB_PIN, OUTPUT);
  motv.init(callback_v);
  moth.init(callback_h);
  disable(CARTESIAN);
#endif

#ifdef DISPLAY_H
  selectSerial3(SERIAL3_CH1, 9600, 100);
  selectSerial3(SERIAL3_CH2, 9600, 100);
#endif

#ifdef GRIPPER_COMMUNICATE_H
  Serial2.begin(9600);   
  Serial2.setTimeout(200);
#endif

//  resetDistNumber(ID_BROADCAST);
//  delay(1000);
//  setDistNumber(channelToId[7][8], 99);
}
 
void loop()
{
//  ping();
  
//  develop(&Serial);

//  Serial.println(getShelf());

  interface(&Serial);

  if (millis()-sleepTimer >= sleepTimeSet && !sleepState)
  {
    sleepState = true;
    int retTarvel = cartesianTravelRelative(0, 0, false, false); 
    disable(CARTESIAN);
    if (retTarvel != REACHED)   return;
  }
     
//  if (DETECT24V)
//  {
//    Serial.println(getTray());
//    delay(100);
//  }
//  else
//  {
//    Serial.println("System: 24v not connected!");
//    delay(500);
//  }
}
