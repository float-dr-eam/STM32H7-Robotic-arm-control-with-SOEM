/*
 * servoS7test.h
 *
 *  Created on: 2025年1月1日
 *      Author: chenlenan
 */

#ifndef SERVOS7TEST_H_
#define SERVOS7TEST_H_

#include "osal.h"
#include <stdio.h>

typedef struct PACKED
{
   uint16 ControlWord;
   int32 TargetPos;
   uint8 TargetMode;
   uint8 var5ffe;
}S7_PDO_Output;

typedef struct PACKED
{
   uint16 StatusWord;
   int32 CurrentPosition;
   int32 CurrentVelocity;
   uint16 ErrorCode;
   uint8 CurrentMode;
   uint8 var5fff;
}S7_PDO_Input;

void servoS7test(char *ifname);
void servoS7_loop(void);

#endif /* SERVOS7TEST_H_ */
