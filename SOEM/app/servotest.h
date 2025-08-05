/*
 * servotest.h
 *
 *  Created on: 2024年12月9日
 *      Author: chenlenan
 */

#ifndef SERVOTEST_H_
#define SERVOTEST_H_
#include "osal.h"
#include <stdio.h>

typedef struct PACKED
{
   uint16 ControlWord;
   int32 TargetPos;
   uint8 TargetMode;
}PDO_Output;

typedef struct PACKED
{
   uint16 StatusWord;
   int32 CurrentPosition;
   int32 CurrentVelocity;
   uint16 ErrorCode;
   uint8 CurrentMode;
}PDO_Input;

void servotest(char *ifname);
void ecat_loop(void);
#endif /* SERVOTEST_H_ */
