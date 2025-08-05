/*
 * za8iotest.h
 *
 *  Created on: 2025年1月1日
 *      Author: chenlenan
 */

#ifndef ZA8IOTEST_H_
#define ZA8IOTEST_H_

#include "osal.h"
#include <stdio.h>

typedef struct PACKED
{
   uint8 var3101;
}ZA8IO_PDO_Output;

typedef struct PACKED
{
   uint8 var3001;


}ZA8IO_PDO_Input;

void za8iotest(char *ifname);
void za8io_loop(void);

#endif /* ZA8IOTEST_H_ */
