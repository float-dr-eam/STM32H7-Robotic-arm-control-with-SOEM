/*
 * DcSync.h
 *
 *  Created on: 2025年1月1日
 *      Author: chenlenan
 */

#ifndef DCSYNC_H_
#define DCSYNC_H_
#include "osal.h"
#include <stdio.h>
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime);

#endif /* DCSYNC_H_ */
