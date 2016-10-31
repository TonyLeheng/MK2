/**
  ******************************************************************************
  * @file	  uArmComm.h
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	  2016-10-08
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#ifndef _UARMCOMM_H_
#define _UARMCOMM_H_

#include <Arduino.h>
#include "uArm.h"
#include "uArmBuzzer.h"

#define COM_LEN_MAX   30

enum CommState
{
  IDLE,
  START,
  CMD,
  END,

  STATE_COUNT
};

class uArmComm
{



public:

 	static void cmdMove(double value[4]);
 	static void cmdMovePol(double value[4]);
 	static void cmdSetAttachServo(double value[4]);
	static void cmdSetDetachServo(double value[4]);
 	static void cmdSetServoAngle(double value[4]);

 	static void cmdSetServoAngleWithOffset(double value[4]);
 	static void cmdSetPump(double value[4]);
 	static void cmdSetGripper(double value[4]);
 	static void cmdSetBuzz(double value[4]);
 	static void cmdStopMove(double value[4]);

	static void cmdGetVersion(double value[4]);
	static void cmdSimulatePos(double value[4]);
	static void cmdGetCurrentXYZ(double value[4]);
 	static void cmdGetCurrentPosPol(double value[4]); 
 	static void cmdGetCurrentAngle(double value[4]);  	

 	static void cmdGetServoAngle(double value[4]);  	
 	static void cmdCoordinateToAngle(double value[4]);  	
 	static void cmdAngleToXYZ(double value[4]);  	
 	static void cmdIsMoving(double value[4]);  	
 	static void cmdGetTip(double value[4]);  	

 	static void cmdGetDigitValue(double value[4]);  	
  	static void cmdSetDigitValue(double value[4]);  	
  	static void cmdGetAnalogValue(double value[4]);  	
  	static void cmdGetE2PROMData(double value[4]);  	
  	static void cmdSetE2PROMData(double value[4]); 

    static void cmdGetGripperStatus(double value[4]);

   
    static void cmdGetPumpStatus(double value[4]);
#ifdef MKII      
    static void cmdGetPowerStatus(double value[4]);
#endif

    static void cmdGetServoAngleData(double value[4]);
    static void cmdGetServoAnalogData(double value[4]); 
    static void cmdWaitReady(double value[4]);
  	static void run();	

	static char parseParam(String cmnd, const char *parameters, int parameterCount, double valueArray[]);
	static void runCommand(String message);
	static void SerialCmdRun();
  static void handleSerialData(char data);

 	static void printf(bool success, double *dat, char *letters, unsigned char num);
	static void printf(bool success, double dat);
	static void printf(bool success, int dat); 	


private:
  static CommState mState;
  static unsigned char cmdReceived[COM_LEN_MAX];
  static unsigned char cmdIndex;
};

#endif // _UARMCOMM_H_
