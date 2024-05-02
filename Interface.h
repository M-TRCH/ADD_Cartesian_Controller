
/*
 *  Interface.h
 *  
 *  Created on: 12 Mar 2024
 *  Author: M.Teerachot
 *  Tested with STM32G431CB
 */

#ifndef INTERFACE_H
#define INTERFACE_H

String packet, cabinet, row, column, dispQt, cmdState, device, tray; 
int cabinetInt, rowInt, columnInt, dispQtInt, cmdStateInt, deviceInt, trayInt; 

int StringToInt(String stringIn, int digit)
{
  int intOut = 0;
  int weight = 1;
  digit = constrain(digit, 1, 4);
  
  for (int cnt=digit-1; cnt>=0; cnt--)
  {
    String stringProc = stringIn.substring(cnt, cnt+1);
         if (stringProc == "1")  intOut += (1 * weight);
    else if (stringProc == "2")  intOut += (2 * weight);
    else if (stringProc == "3")  intOut += (3 * weight);
    else if (stringProc == "4")  intOut += (4 * weight);
    else if (stringProc == "5")  intOut += (5 * weight);
    else if (stringProc == "6")  intOut += (6 * weight);
    else if (stringProc == "7")  intOut += (7 * weight);
    else if (stringProc == "8")  intOut += (8 * weight);
    else if (stringProc == "9")  intOut += (9 * weight); 
    weight *= 10;
  }
  return intOut;
}

#endif
