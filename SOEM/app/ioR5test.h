/*
 * ioR5test.h
 *
 *  Created on: 2025年1月1日
 *      Author: chenlenan
 */

#ifndef IOR5TEST_H_
#define IOR5TEST_H_

#include "osal.h"
#include <stdio.h>

typedef struct PACKED
{
   uint8 var7010;
   uint8 pad0;
}IOR5_PDO_Output;

typedef struct PACKED
{
   uint8 var6000;
   uint8 pad1;
   uint16 var6020;
   uint16 ai;

}IOR5_PDO_Input;

void ioR5test(char *ifname);
void ioR5_loop(void);

#endif /* IOR5TEST_H_ */
