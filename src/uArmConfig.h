/**
  ******************************************************************************
  * @file	uArmConfig.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMCONFIG_H_
#define _UARMCONFIG_H_

#include <Arduino.h>

int ardprintf(char *result, char *str, ...);
#define MKII
//#define METAL

//#define DEBUG


//#define METAL_MOTOR

#ifdef DEBUG
	#define debugPrint	dprint
#else
	#define debugPrint
#endif

#define current_ver         "[SH2-2.1.3]"

#define TICK_INTERVAL    50    // ms


// conver double value to string
char* D(double value);

#ifdef DEBUG

void dprint(char *fmt, ...);

#ifdef F 
void dprint(const __FlashStringHelper *fmt, ...);
#endif

#else

#endif // DEBUG


#endif // _UARMCONFIG_H_
