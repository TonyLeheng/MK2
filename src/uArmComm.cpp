/**
  ******************************************************************************
  * @file	gCommComm.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-10-08
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArmComm.h" 

uArmComm gComm;

// unsigned char uArmComm::cmdIndex = 0;
// CommState uArmComm::mState = IDLE;
// unsigned char uArmComm::cmdReceived[COM_LEN_MAX];

/*
struct Command
{
	char cmd[5];
	int parametersCount;
	char parameters[4];
	void (*execute)(double value[4]);
};

const Command command[] PROGMEM= {
	{"sMov", 4, {'X', 'Y', 'Z', 'V'}, &gComm.cmdMove},     // 
	{"sPol", 4, {'S', 'R', 'H', 'V'}, &gComm.cmdMovePol},  // 
	{"sAtt", 1, {'N'}, &gComm.cmdSetAttachServo},          // 
	{"sDet", 1, {'N'}, &gComm.cmdSetDetachServo},          // 
	{"sSer", 2, {'N', 'V'}, &gComm.cmdSetServoAngle},      

	{"sAng", 2, {'N', 'V'}, &gComm.cmdSetServoAngleWithOffset},    // 
	{"sPum", 1, {'V'}, &gComm.cmdSetPump},                 // 
	{"sGri", 1, {'V'}, &gComm.cmdSetGripper},              // 
	{"sBuz", 2, {'F', 'T'}, &gComm.cmdSetBuzz},            // 
	{"sStp", 0, {}, &gComm.cmdStopMove},                   // 

	{"gVer", 0, {}, &gComm.cmdGetVersion},                 // 
	{"gSim", 3, {'X', 'Y', 'Z'}, &gComm.cmdSimulatePos},   // 
	{"gCrd", 0, {}, &gComm.cmdGetCurrentXYZ},              // 
	{"gPol", 0, {}, &gComm.cmdGetCurrentPosPol},           // 
	{"gAng", 0, {}, &gComm.cmdGetCurrentAngle},            // 

	{"gSer", 0, {}, &gComm.cmdGetServoAngle},              
	{"gIKX", 3, {'X', 'Y', 'Z'}, &gComm.cmdCoordinateToAngle}, //
	{"gFKT", 3, {'T', 'L', 'R'}, &gComm.cmdAngleToXYZ},        // 
	{"gMov", 0, {}, &gComm.cmdIsMoving},                       // 
	{"gTip", 0, {}, &gComm.cmdGetTip},                         // 

	{"gDig", 1, {'N'}, &gComm.cmdGetDigitValue},               // 
	{"sDig", 2, {'N', 'V'}, &gComm.cmdSetDigitValue},          // 
	{"gAna", 1, {'N'}, &gComm.cmdGetAnalogValue},              // 
	{"gEEP", 2, {'A', 'T'}, &gComm.cmdGetE2PROMData},          // 
	{"sEEP", 3, {'A', 'T', 'V'}, &gComm.cmdSetE2PROMData},     // 


    {"gGri", 0, {}, &gComm.cmdGetGripperStatus},                // 
    {"gPum", 0, {}, &gComm.cmdGetPumpStatus},                   // 

#ifdef MKII     
    {"gPow", 0, {}, &gComm.cmdGetPowerStatus},                  // 
#endif

    {"gSAD", 1, {'N'}, &gComm.cmdGetServoAnalogData}

    
	
};

*/
uArmComm::uArmComm()
{
    cmdIndex = 0;
    mState = IDLE;
}



void uArmComm::replyNoCmd(int serialNum)
{
    Serial.print("$");
    Serial.print(serialNum);
    Serial.print(" E");
    Serial.println(NO_SUCH_CMD);   
}

void uArmComm::replyOK(int serialNum)
{
    Serial.print("$");
    Serial.print(serialNum);
    Serial.print(" ");
    Serial.println("OK");   
}

void uArmComm::replyResult(int serialNum, String result)
{
    Serial.print("$");
    Serial.print(serialNum);
    Serial.print(" OK ");
    Serial.println(result);   
}

unsigned char uArmComm::cmdMove(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 4)
        return PARAMETER_ERROR;

	debugPrint("cmdMove x:%s, y:%s, z:%s, v:%s\n\n", D(value[0]), D(value[1]), D(value[2]), D(value[3]));


	if (uArm.moveTo(value[0], value[1], value[2], value[3]) != OUT_OF_RANGE_NO_SOLUTION)
	{
        replyOK(serialNum);
	}
	else
	{
		return OUT_OF_RANGE;
	}

    return 0;
}

unsigned char uArmComm::cmdMovePol(int serialNum, int parameterCount, double value[4])
{
	double x, y, z;

    if (parameterCount != 4)
        return PARAMETER_ERROR;


	uArm.mController.getXYZFromPolar(x, y, z, value[0], value[1], value[2]);
	uArm.moveTo(x, y, z, value[3]);	

    replyOK(serialNum);
    return 0;   
}

unsigned char uArmComm::cmdSetAttachServo(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 1)
        return PARAMETER_ERROR;

	uArm.mController.attachServo(value[0]);

    replyOK(serialNum);
    return 0;
}

unsigned char uArmComm::cmdSetDetachServo(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 1)
        return PARAMETER_ERROR;

	uArm.mController.detachServo(value[0]);
    replyOK(serialNum);

    return 0;
}

unsigned char uArmComm::cmdSetServoAngle(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 2)
        return PARAMETER_ERROR;

	uArm.mController.writeServoAngle(byte(value[0]), value[1], false);
    replyOK(serialNum);

    return 0;
}

unsigned char uArmComm::cmdSetServoAngleWithOffset(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 2)
        return PARAMETER_ERROR;    

	uArm.mController.writeServoAngle(byte(value[0]), value[1], true);
    replyOK(serialNum);

    return 0;

}

unsigned char uArmComm::cmdSetPump(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 1)
        return PARAMETER_ERROR;

    if (value[0] == 0)//off
    {
        uArm.mController.pumpOff();
    }
    else//on
    {
        uArm.mController.pumpOn();
    }

    replyOK(serialNum);

    return 0;
}

unsigned char uArmComm::cmdSetGripper(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 1)
        return PARAMETER_ERROR;

    if (value[0]==0)//release
    {
        uArm.mController.gripperRelease();
    }
    else//catch
    {
        uArm.mController.gripperCatch();
    }

    replyOK(serialNum);

    return 0;
}

unsigned char uArmComm::cmdSetBuzz(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 2)
        return PARAMETER_ERROR;

	gBuzzer.buzz(value[0], value[1]*1000);    // convert to ms

    replyOK(serialNum);

    return 0;
}

unsigned char uArmComm::cmdStopMove(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

	uArm.stopMove();
    replyOK(serialNum);

    return 0;
}

unsigned char uArmComm::cmdGetVersion(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;    

    char result[128];

    ardprintf(result, "V%s", current_ver);


	replyResult(serialNum, result);

    return 0;
}

unsigned char uArmComm::cmdSimulatePos(int serialNum, int parameterCount, double value[4])
{
	double angleRot, angleLeft, angleRight;

    if (parameterCount != 4)
        return PARAMETER_ERROR;

    if (value[3] == 1) //Polar
    {
        double s = value[0];
        double r = value[1];
        double h = value[2];

        uArm.mController.getXYZFromPolar(value[0], value[1], value[2], s, r, h);
    }

	unsigned char status = uArm.mController.coordianteToAngle(value[0], value[1], value[2], angleRot, angleLeft, angleRight, false);


    char result[128];
    switch(status)
    {
    case IN_RANGE: 
    	strcpy(result, "R1");
        break;

    case OUT_OF_RANGE: 
    case OUT_OF_RANGE_NO_SOLUTION: 
    	strcpy(result, "R0");
       	break;
    default:                
    	break;
    }

    replyResult(serialNum, result);

    return 0;
}

unsigned char uArmComm::cmdGetCurrentXYZ(int serialNum, int parameterCount, double value[4])
{
    //char letters[3] = {'X','Y','Z'};

    if (parameterCount != 0)
        return PARAMETER_ERROR;
    //debugPrint("cmdGetCurrentXYZ");

    uArm.mController.updateAllServoAngle();
    uArm.mController.getCurrentXYZ(value[0], value[1], value[2]);
    
    char result[128];
    ardprintf(result, "X%f Y%f Z%f", value[0], value[1], value[2]);

    replyResult(serialNum, result);
    return 0;	
}

unsigned char uArmComm::cmdGetCurrentPosPol(int serialNum, int parameterCount, double value[4])
{
	double angleRot, angleLeft, angleRight;
	double x, y, z;

    if (parameterCount != 0)
        return PARAMETER_ERROR;


    uArm.mController.updateAllServoAngle();
	uArm.mController.getCurrentXYZ(x, y, z);
	uArm.mController.getServoAngles(angleRot, angleLeft, angleRight);
    double stretch;
    stretch = sqrt(x * x + y * y);
    //char letters[3] = {'S','R','H'};
    value[0] = stretch;
    value[1] = angleRot;
    value[2] = z;
    //printf(true, value, letters, 3);

    char result[128];
    ardprintf(result, "S%f R%f H%f", value[0], value[1], value[2]);

    replyResult(serialNum, result);

    return 0;
}

unsigned char uArmComm::cmdGetCurrentAngle(int serialNum, int parameterCount, double value[4])
{
    //char letters[4] = {'B','L','R','H'};

    if (parameterCount != 0)
        return PARAMETER_ERROR;

    value[0] = uArm.mController.readServoAngle(SERVO_ROT_NUM, true);
    value[1] = uArm.mController.readServoAngle(SERVO_LEFT_NUM, true);
    value[2] = uArm.mController.readServoAngle(SERVO_RIGHT_NUM, true);
    value[3] = uArm.mController.readServoAngle(SERVO_HAND_ROT_NUM, true);
    //printf(true, value, letters, 4);

    char result[128];
    ardprintf(result, "B%f L%f R%f H%f", value[0], value[1], value[2], value[3]);

    replyResult(serialNum, result);

    return 0;
}

unsigned char uArmComm::cmdGetServoAngle(int serialNum, int parameterCount, double value[4])
{
    //char letters[4] = {'B','L','R','H'};

    if (parameterCount != 0)
        return PARAMETER_ERROR;

    value[0] = uArm.mController.readServoAngle(SERVO_ROT_NUM, false);
    value[1] = uArm.mController.readServoAngle(SERVO_LEFT_NUM, false);
    value[2] = uArm.mController.readServoAngle(SERVO_RIGHT_NUM, false);
    value[3] = uArm.mController.readServoAngle(SERVO_HAND_ROT_NUM, false);
    //printf(true, value, letters, 4);

    char result[128];
    ardprintf(result, "B%f L%f R%f H%f", value[0], value[1], value[2], value[3]);

    replyResult(serialNum, result);

    return 0;
}

unsigned char uArmComm::cmdCoordinateToAngle(int serialNum, int parameterCount, double value[4])
{
    double rot, left, right;
    //bool success;

    if (parameterCount != 3)
        return PARAMETER_ERROR;

    uArm.mController.coordianteToAngle(value[0], value[1], value[2], rot, left, right);
    //char letters[3] = {'T','L','R'};
    value[0] = rot;
    value[1] = left;
    value[2] = right;
    //success = true;
    //printf(success,value,letters,3);
    char result[128];
    ardprintf(result, "B%f L%f R%f", value[0], value[1], value[2]);

    replyResult(serialNum, result);


    return 0;
}

unsigned char uArmComm::cmdAngleToXYZ(int serialNum, int parameterCount, double value[4])
{
    double x, y, z;
    bool success;

    if (parameterCount != 3)
        return PARAMETER_ERROR;

    if(uArm.mController.getXYZFromAngle(x, y, z, value[0], value[1], value[2]) == OUT_OF_RANGE)
    {
        success = false;
    }
    else
    {
        success = true;
    }

    //char letters[3] = {'X','Y','Z'};
    value[0] = x;
    value[1] = y;
    value[2] = z;

    //printf(success,value,letters,3);
    char result[128];
    ardprintf(result, "X%f Y%f Z%f", value[0], value[1], value[2]);

    replyResult(serialNum, result);
    return 0;
}

unsigned char uArmComm::cmdIsMoving(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[128];
	if(uArm.isMoving())
	{
        strcpy(result, "S1");
	}
	else
	{
        strcpy(result, "S0");
	}

    replyResult(serialNum, result);

    return 0;
}

unsigned char uArmComm::cmdGetTip(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[128];
    if(digitalRead(LIMIT_SW))
    {
        strcpy(result, "R1");
    }
    else
    {
        strcpy(result, "R0");
    }

     replyResult(serialNum, result);

    return 0;
}

unsigned char uArmComm::cmdGetDigitValue(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 1)
        return PARAMETER_ERROR;

    int val = digitalRead(value[0]);
   
    //printf(true, val);
    char result[128];
    ardprintf(result, "R%d", val);

    replyResult(serialNum, result);
    return 0;
}

unsigned char uArmComm::cmdSetDigitValue(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 2)
        return PARAMETER_ERROR;

    //Serial.println(SS);// successful feedback send it immediately
    // write the digit value
    value[1] == 1 ? digitalWrite(value[0], HIGH) : digitalWrite(value[0], LOW);

    replyOK(serialNum);
    return 0;
}

unsigned char uArmComm::cmdGetAnalogValue(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 1)
        return PARAMETER_ERROR;

    int val = analogRead(value[0]);

    //printf(true, val);
    char result[128];
    ardprintf(result, "R%d", val);

    replyResult(serialNum, result);
    return 0;
}

unsigned char uArmComm::cmdGetE2PROMData(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 2)
        return PARAMETER_ERROR;    

    char result[128];
    switch(int(value[1]))
    {
    case DATA_TYPE_BYTE:
    	{
            int val = EEPROM.read(value[0]);
            ardprintf(result, "R%d", val);
            break;
    	}
    case DATA_TYPE_INTEGER:
    	{
            int i_val = 0;
            EEPROM.get(value[0], i_val);
            ardprintf(result, "R%d", i_val);
            //Serial.println("S" + String(i_val) + "");
            break;
    	}
    case DATA_TYPE_FLOAT:
    	{
            double f_val = 0.0f;
            EEPROM.get(value[0],f_val);
            ardprintf(result, "R%f", f_val);
            //Serial.println("S" + String(f_val) + "");
            break;
    	}
    }

    
    

    replyResult(serialNum, result);    

    return 0;

}

unsigned char uArmComm::cmdSetE2PROMData(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 3)
        return PARAMETER_ERROR;    

   // Serial.println(SS);// successful feedback send it immediately
    // write the EEPROM value
    switch(int(value[1]))
    {
    case DATA_TYPE_BYTE:
    	{
            byte b_val;
            b_val = byte(value[2]);
            EEPROM.write(value[0], b_val);
            break;
    	}
    case DATA_TYPE_INTEGER:
    	{
            int i_val = 0;
            i_val = int(value[2]);
            EEPROM.put(value[0], i_val);
            break;
    	}
    case DATA_TYPE_FLOAT:
    	{
    	    float f_val = 0.0f;
            f_val = float(value[2]);
            EEPROM.put(value[0],f_val);
            // Serial.println(f_val);
            break;
    	}
    }

    replyOK(serialNum);

    return 0;

}

unsigned char uArmComm::cmdGetGripperStatus(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    unsigned char status = uArm.mController.gripperStatus();

    //String ret = "[S" + String(status) + "]";

    //Serial.println(ret);
    char result[128];
    ardprintf(result, "R%d", status);
    replyResult(serialNum, result);
    return 0;
}


unsigned char uArmComm::cmdGetPumpStatus(int serialNum, int parameterCount, double value[4])
{

    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[128];
#ifdef MKII     
    

    unsigned char status = uArm.mController.pumpStatus();
 
    //String ret = "[S" + String(status) + "]";

    //Serial.println(ret);
    ardprintf(result, "R%d", status);

#elif defined(METAL)

    if (uArm.mController.pumpStatus())
        strcpy(result, "R1");
    else
        strcpy(result, "R0");

#endif

    replyResult(serialNum, result);

    return 0;
}


#ifdef MKII 
unsigned char uArmComm::cmdGetPowerStatus(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 0)
        return PARAMETER_ERROR;

    char result[128];

    if (uArm.isPowerPlugIn())
        strcpy(result, "R1");
    else
        strcpy(result, "R0");   

    replyResult(serialNum, result);

    return 0;
}
#endif 

unsigned char uArmComm::cmdGetServoAnalogData(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 1)
        return PARAMETER_ERROR;

    unsigned int data = uArm.mController.getServoAnalogData(value[0]);
    //Serial.println(data);
    char result[128];


    ardprintf(result, "R%d", result);   

    replyResult(serialNum, result);

    return 0;
}


unsigned char uArmComm::cmdRelativeMove(int serialNum, int parameterCount, double value[4])
{
    if (parameterCount != 4)
        return PARAMETER_ERROR;

    debugPrint("cmdRelativeMove x:%s, y:%s, z:%s, v:%s\n\n", D(value[0]), D(value[1]), D(value[2]), D(value[3]));

    double x, y, z;
    // get cur xyz
    uArm.mController.getCurrentXYZ(x, y, z);

    value[0] += x;
    value[1] += y;
    value[2] += z;

    if (uArm.moveTo(value[0], value[1], value[2], value[3]) != OUT_OF_RANGE_NO_SOLUTION)
    {
        replyOK(serialNum);
    }
    else
    {
        return OUT_OF_RANGE;
    }

    return 0;
}

/*
char uArmComm::parseParam(String cmnd, const char *parameters, int parameterCount, double valueArray[])
{
    for (byte i = 0; i < parameterCount; i++) 
    {
        char startIndex = cmnd.indexOf(parameters[i]) + 1;
        //debugPrint("startIndex = %d", startIndex);
        if (startIndex == -1)
        {
            //Serial.println(F("[F1]"));
            return ERR1;
        }

        char endIndex = 0;
        if (i != parameterCount-1) 
        {
            endIndex = cmnd.indexOf(parameters[i+1]);
            if (endIndex == -1)
            {
                //Serial.println(F("[F1]"));
                return ERR1;
            }
        }
        else
        {
            endIndex = cmnd.length();
        }
       
        valueArray[i] = cmnd.substring(startIndex, endIndex).toFloat();

    }



    return OK;
}


void uArmComm::runCommand(String message)
{
	
	String cmdStr = message.substring(0, 4);
	double value[4];
    int i = 0;

	Command iCmd;

	int len = sizeof(command)/sizeof(command[0]);

	for (i = 0; i < len; i++)
	{
		memcpy_P(&iCmd, &command[i], sizeof(Command));
		
		if (cmdStr == iCmd.cmd)
		{

			if (iCmd.parametersCount == 0)
			{
				iCmd.execute(value);
			}
			else if (parseParam(message, iCmd.parameters, iCmd.parametersCount, value) == OK) 
			{   
				iCmd.execute(value);
            }	
            else
            {
                Serial.println("[ERR1]");
            }
            break;
		}
		
	}

    if (i == len)
    {
        Serial.println("[ERR3]");
    }
	
}
*/


void uArmComm::HandleMoveCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
    unsigned char result = false;

    switch (cmdCode)
    {
    case 0:
        result = cmdMove(serialNum, parameterCount, value);
        break;

    case 201:
        result = cmdMovePol(serialNum, parameterCount, value);
        break;

    case 202:
        result = cmdSetServoAngleWithOffset(serialNum, parameterCount, value);
        break;

    case 203:
        result = cmdStopMove(serialNum, parameterCount, value);
        break;

    case 204:
        result = cmdRelativeMove(serialNum, parameterCount, value);
        break;
        
    default:
        replyNoCmd(serialNum);
        return;
    }

    if (result > 0)
    {
        Serial.print("$");
        Serial.print(serialNum);
        Serial.print(" ");
        Serial.print("E");
        Serial.println(result);
    }
}

void uArmComm::HandleSettingCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
    unsigned char result = false;

    switch (cmdCode)
    {
    case 200:
        result = cmdIsMoving(serialNum, parameterCount, value);
        break;

    case 201:
        result = cmdSetAttachServo(serialNum, parameterCount, value);
        break;

    case 202:
        result = cmdSetDetachServo(serialNum, parameterCount, value);
        break;

    case 210:
        result = cmdSetBuzz(serialNum, parameterCount, value);
        break;

    case 220:
        result = cmdCoordinateToAngle(serialNum, parameterCount, value);
        break;

    case 221:
        result = cmdAngleToXYZ(serialNum, parameterCount, value);
        break;

    case 222:
        result = cmdSimulatePos(serialNum, parameterCount, value);
        break;

    case 231:
        result = cmdSetPump(serialNum, parameterCount, value);
        break;

    case 232:
        result = cmdSetGripper(serialNum, parameterCount, value);
        break;

    case 240:
        result = cmdSetDigitValue(serialNum, parameterCount, value);
        break;



    default:
        replyNoCmd(serialNum);
        return;
    }

    if (result > 0)
    {
        Serial.print("$");
        Serial.print(serialNum);
        Serial.print(" ");
        Serial.print("E");
        Serial.println(result);
    }
}

void uArmComm::HandleQueryCmd(int cmdCode, int serialNum, int parameterCount, double value[4])
{
    unsigned char result = false;

    switch (cmdCode)
    {
    case 200:
        result = cmdGetCurrentAngle(serialNum, parameterCount, value);
        break;

    case 201:
        result = cmdGetVersion(serialNum, parameterCount, value);
        break;

    case 220:
        result = cmdGetCurrentXYZ(serialNum, parameterCount, value);
        break;

    case 221:
        result = cmdGetCurrentPosPol(serialNum, parameterCount, value);
        break;

    case 231:
        result = cmdGetPumpStatus(serialNum, parameterCount, value);
        break;

    case 232:
        result = cmdGetGripperStatus(serialNum, parameterCount, value);
        break;

    case 240:
        result = cmdGetDigitValue(serialNum, parameterCount, value);
        break;

    case 241:
        result = cmdGetAnalogValue(serialNum, parameterCount, value);
        break;

    default:
        replyNoCmd(serialNum);
        return;
    }

    if (result > 0)
    {
        Serial.print("$");
        Serial.print(serialNum);
        Serial.print(" ");
        Serial.print("E");
        Serial.println(result);
    }
}

bool uArmComm::parseCommand(char *message)
{
    double value[6];
    int index = 0;

    //Serial.println(message);


    int len = strlen(message);

    char *pch;
    char cmdType;

    while (len > 0)
    {
        if (isspace(message[len-1]))
        {
            message[len-1] = '\0';
        }
        else
        {
            break;
        }

        len--;
    }

    if (len <= 0)
        return false;



    pch = strtok(message, " ");
    while (pch != NULL && index < 6)
    {
        //Serial.println(pch);
        
        switch (index)
        {
        case 0:
            value[index] = atof(pch);
            break;

        case 1:
            cmdType = *(pch);
            //Serial.println(cmdType);
            value[index] = atof(pch+1);
            break;

        default:
            value[index] = atof(pch+1);
            break;
        }


        //Serial.print("value=");
        //Serial.println(value[index]);

        pch = strtok(NULL, " ");


        index++;

    }

    int serialNum = value[0];
    int cmdCode = value[1];
    int parameterCount = index - 2;

    switch (cmdType)
    {
    case 'G':
        HandleMoveCmd(cmdCode, serialNum, parameterCount, value+2);
        break;

    case 'M':
        HandleSettingCmd(cmdCode, serialNum, parameterCount, value+2);
        break;

    case 'P':
        HandleQueryCmd(cmdCode, serialNum, parameterCount, value+2);
        break;

    }




}

void uArmComm::handleSerialData(char data)
{
    switch (mState)
    {
    case IDLE:
        if (data == '#')
        {
            mState = START;
            cmdIndex = 0;

        }
        break;
        
    case START:
        
        if (cmdIndex >= COM_LEN_MAX)
        {

            mState = IDLE;
        }
        else if (data == '#')
        {

            cmdIndex = 0;
        }
        else if (data == '\n')
        {

            cmdReceived[cmdIndex] = '\0';

            parseCommand((char*)cmdReceived);
            mState = IDLE;
        }
        else
        {

            cmdReceived[cmdIndex++] = data;
        }
        break;
        
    default:
        mState = IDLE;
        break;
              
    }
}

void uArmComm::SerialCmdRun()
{

    char data = -1;

    while (Serial.available())
    {
        data = Serial.read();

        if (data == -1)
        {
            return ;
        }
        else
        {
            handleSerialData(data);
        }
    }
}

void uArmComm::run()
{
	SerialCmdRun();
}

/*
void uArmComm::printf(bool success, double *dat, char *letters, unsigned char num)
{
    if(success == true)
        Serial.print(F("[S"));
    else
        Serial.print(F("[F"));
    //print the parameter
    for(unsigned char i = 0; i < num; i++)
    {
        Serial.print(letters[i]);
        Serial.print(dat[i]);
    }
    Serial.println(F("]"));

}

void uArmComm::printf(bool success, double dat)
{
    if(success == true)
        Serial.print(F("[S"));
    else
        Serial.print(F("[F"));

    Serial.print(dat);
    Serial.println(F("]"));

}

void uArmComm::printf(bool success, int dat)
{
    if(success == true)
        Serial.print(F("[S"));
    else
        Serial.print(F("[F"));

    Serial.print(dat);
    Serial.println(F("]"));
}
*/
