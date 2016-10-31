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

unsigned char uArmComm::cmdIndex = 0;
CommState uArmComm::mState = IDLE;
unsigned char uArmComm::cmdReceived[COM_LEN_MAX];

struct Command
{
	char cmd[5];
	int parametersCount;
	char parameters[4];
	void (*execute)(double value[4]);
};

const Command command[] PROGMEM= {
	{"sMov", 4, {'X', 'Y', 'Z', 'V'}, &gComm.cmdMove},
	{"sPol", 4, {'S', 'R', 'H', 'V'}, &gComm.cmdMovePol},
	{"sAtt", 1, {'N'}, &gComm.cmdSetAttachServo},
	{"sDet", 1, {'N'}, &gComm.cmdSetDetachServo},
	{"sSer", 2, {'N', 'V'}, &gComm.cmdSetServoAngle},

	{"sAng", 2, {'N', 'V'}, &gComm.cmdSetServoAngleWithOffset},
	{"sPum", 1, {'V'}, &gComm.cmdSetPump},
	{"sGri", 1, {'V'}, &gComm.cmdSetGripper},
	{"sBuz", 2, {'F', 'T'}, &gComm.cmdSetBuzz},
	{"sStp", 0, {}, &gComm.cmdStopMove},

	{"gVer", 0, {}, &gComm.cmdGetVersion},
	{"gSim", 3, {'X', 'Y', 'Z'}, &gComm.cmdSimulatePos},
	{"gCrd", 0, {}, &gComm.cmdGetCurrentXYZ},
	{"gPol", 0, {}, &gComm.cmdGetCurrentPosPol},
	{"gAng", 0, {}, &gComm.cmdGetCurrentAngle},

	{"gSer", 0, {}, &gComm.cmdGetServoAngle},
	{"gIKX", 3, {'X', 'Y', 'Z'}, &gComm.cmdCoordinateToAngle},
	{"gFKT", 3, {'T', 'L', 'R'}, &gComm.cmdAngleToXYZ},
	{"gMov", 0, {}, &gComm.cmdIsMoving},
	{"gTip", 0, {}, &gComm.cmdGetTip},

	{"gDig", 1, {'N'}, &gComm.cmdGetDigitValue},
	{"sDig", 2, {'N', 'V'}, &gComm.cmdSetDigitValue},
	{"gAna", 1, {'N'}, &gComm.cmdGetAnalogValue},
	{"gEEP", 2, {'A', 'T'}, &gComm.cmdGetE2PROMData},
	{"sEEP", 3, {'A', 'T', 'V'}, &gComm.cmdSetE2PROMData},


    {"gGri", 0, {}, &gComm.cmdGetGripperStatus},
    {"gPum", 0, {}, &gComm.cmdGetPumpStatus},

#ifdef MKII     
    {"gPow", 0, {}, &gComm.cmdGetPowerStatus},
#endif

    {"gSAD", 1, {'N'}, &gComm.cmdGetServoAnalogData}

    
	
};



void uArmComm::cmdMove(double value[4])
{

	debugPrint("cmdMove x:%s, y:%s, z:%s, v:%s\n\n", D(value[0]), D(value[1]), D(value[2]), D(value[3]));


	if (uArm.moveTo(value[0], value[1], value[2], value[3]) != OUT_OF_RANGE_NO_SOLUTION)
	{
		Serial.println(SS);
	}
	else
	{
		Serial.println(FF);
	}
}

void uArmComm::cmdMovePol(double value[4])
{
	double x, y, z;

	Serial.println(SS);
	uArm.mController.getXYZFromPolar(x, y, z, value[0], value[1], value[2]);
	uArm.moveTo(x, y, z, value[3]);	
}

void uArmComm::cmdSetAttachServo(double value[4])
{
	Serial.println(SS);// successful feedback send it immediately
	uArm.mController.attachServo(value[0]);
}

void uArmComm::cmdSetDetachServo(double value[4])
{
	Serial.println(SS);// successful feedback send it immediately
	uArm.mController.detachServo(value[0]);
}

void uArmComm::cmdSetServoAngle(double value[4])
{
	Serial.println(SS);// successful feedback send it immediately
	uArm.mController.writeServoAngle(byte(value[0]), value[1], false);
}

void uArmComm::cmdSetServoAngleWithOffset(double value[4])
{
	Serial.println(SS);// successful feedback send it immediately

    //double angle = uArm.mController.getReverseServoAngle(byte(value[0]), value[1]);
    double angle = value[1];
	uArm.mController.writeServoAngle(byte(value[0]), angle, true);

}

void uArmComm::cmdSetPump(double value[4])
{
	Serial.println(SS);// successful feedback send it immediately
    if (value[0] == 0)//off
    {
        uArm.mController.pumpOff();
    }
    else//on
    {
        uArm.mController.pumpOn();
    }
}

void uArmComm::cmdSetGripper(double value[4])
{
	Serial.println(SS);// successful feedback send it immediately
    if (value[0]==0)//release
    {
        uArm.mController.gripperRelease();
    }
    else//catch
    {
        uArm.mController.gripperCatch();
    }
}

void uArmComm::cmdSetBuzz(double value[4])
{
	Serial.println(SS);// successful feedback send it immediately
	gBuzzer.buzz(value[0], value[1]*1000);    // convert to ms
}

void uArmComm::cmdStopMove(double value[4])
{
	Serial.println(SS);// successful feedback send it immediately
	uArm.stopMove();
}

void uArmComm::cmdGetVersion(double value[4])
{
	Serial.println(current_ver);
}

void uArmComm::cmdSimulatePos(double value[4])
{
	double angleRot, angleLeft, angleRight;

    if (value[3] == 1) //Polar
    {
        double s = value[0];
        double r = value[1];
        double h = value[2];

        uArm.mController.getXYZFromPolar(value[0], value[1], value[2], s, r, h);
    }

	unsigned char status = uArm.mController.coordianteToAngle(value[0], value[1], value[2], angleRot, angleLeft, angleRight, false);



    switch(status)
    {
    case IN_RANGE: 
    	Serial.println(S0);
        break;
    case OUT_OF_RANGE: 
    	Serial.println(F0);
        break;
    case OUT_OF_RANGE_NO_SOLUTION: 
    	Serial.println(F1);
       	break;
    default:                
    	break;
    }
}

void uArmComm::cmdGetCurrentXYZ(double value[4])
{
    char letters[3] = {'X','Y','Z'};

    //debugPrint("cmdGetCurrentXYZ");

    uArm.mController.updateAllServoAngle();
    uArm.mController.getCurrentXYZ(value[0], value[1], value[2]);
    printf(true, value, letters, 3);	
}

void uArmComm::cmdGetCurrentPosPol(double value[4])
{
	double angleRot, angleLeft, angleRight;
	double x, y, z;

    uArm.mController.updateAllServoAngle();
	uArm.mController.getCurrentXYZ(x, y, z);
	uArm.mController.getServoAngles(angleRot, angleLeft, angleRight);
    double stretch;
    stretch = sqrt(x * x + y * y);
    char letters[3] = {'S','R','H'};
    value[0] = stretch;
    value[1] = angleRot;
    value[2] = z;
    printf(true, value, letters, 3);
}

void uArmComm::cmdGetCurrentAngle(double value[4])
{
    char letters[4] = {'B','L','R','H'};
    value[0] = uArm.mController.readServoAngle(SERVO_ROT_NUM, true);
    value[1] = uArm.mController.readServoAngle(SERVO_LEFT_NUM, true);
    value[2] = uArm.mController.readServoAngle(SERVO_RIGHT_NUM, true);
    value[3] = uArm.mController.readServoAngle(SERVO_HAND_ROT_NUM, true);
    printf(true, value, letters, 4);
}

void uArmComm::cmdGetServoAngle(double value[4])
{
    char letters[4] = {'B','L','R','H'};
    value[0] = uArm.mController.readServoAngle(SERVO_ROT_NUM, false);
    value[1] = uArm.mController.readServoAngle(SERVO_LEFT_NUM, false);
    value[2] = uArm.mController.readServoAngle(SERVO_RIGHT_NUM, false);
    value[3] = uArm.mController.readServoAngle(SERVO_HAND_ROT_NUM, false);
    printf(true, value, letters, 4);
}

void uArmComm::cmdCoordinateToAngle(double value[4])
{
    double rot, left, right;
    bool success;

    uArm.mController.coordianteToAngle(value[0], value[1], value[2], rot, left, right);
    char letters[3] = {'T','L','R'};
    value[0] = rot;
    value[1] = left;
    value[2] = right;
    success = true;
    printf(success,value,letters,3);
}

void uArmComm::cmdAngleToXYZ(double value[4])
{
    double x, y, z;
    bool success;

    if(uArm.mController.getXYZFromAngle(x, y, z, value[0], value[1], value[2]) == OUT_OF_RANGE)
    {
        success = false;
    }
    else
    {
        success = true;
    }

    char letters[3] = {'X','Y','Z'};
    value[0] = x;
    value[1] = y;
    value[2] = z;

    printf(success,value,letters,3);
}

void uArmComm::cmdIsMoving(double value[4])
{
	if(uArm.isMoving())
	{
        Serial.println(SS);
	}
	else
	{
        Serial.println(FF);
	}
}

void uArmComm::cmdGetTip(double value[4])
{
    if(digitalRead(LIMIT_SW))
    {
        Serial.println(S0);
    }
    else
    {
        Serial.println(S1);
    }
}

void uArmComm::cmdGetDigitValue(double value[4])
{
    int val = digitalRead(value[0]);
   
    printf(true, val);
}

void uArmComm::cmdSetDigitValue(double value[4])
{
    Serial.println(SS);// successful feedback send it immediately
    // write the digit value
    value[1] == 1 ? digitalWrite(value[0], HIGH) : digitalWrite(value[0], LOW);
}

void uArmComm::cmdGetAnalogValue(double value[4])
{
    int val = analogRead(value[0]);

    printf(true, val);
}

void uArmComm::cmdGetE2PROMData(double value[4])
{
    
    switch(int(value[1]))
    {
    case DATA_TYPE_BYTE:
    	{
            int val = EEPROM.read(value[0]);
            printf(true, val);
            break;
    	}
    case DATA_TYPE_INTEGER:
    	{
            int i_val = 0;
            EEPROM.get(value[0], i_val);
            printf(true, i_val);
            //Serial.println("S" + String(i_val) + "");
            break;
    	}
    case DATA_TYPE_FLOAT:
    	{
            double f_val = 0.0f;
            EEPROM.get(value[0],f_val);
            printf(true, f_val);
            //Serial.println("S" + String(f_val) + "");
            break;
    	}
    }

}

void uArmComm::cmdSetE2PROMData(double value[4])
{

    Serial.println(SS);// successful feedback send it immediately
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

}

void uArmComm::cmdGetGripperStatus(double value[4])
{
    unsigned char status = uArm.mController.gripperStatus();

    String ret = "[S" + String(status) + "]";

    Serial.println(ret);
}


void uArmComm::cmdGetPumpStatus(double value[4])
{
#ifdef MKII     
    
    unsigned char status = uArm.mController.pumpStatus();

    String ret = "[S" + String(status) + "]";

    Serial.println(ret);

#elif defined(METAL)

    if (uArm.mController.pumpStatus())
        Serial.println(SS);
    else
        Serial.println(FF);   

#endif
}


#ifdef MKII 
void uArmComm::cmdGetPowerStatus(double value[4])
{
    if (uArm.isPowerPlugIn())
        Serial.println(SS);
    else
        Serial.println(FF);    
}
#endif 

void uArmComm::cmdGetServoAnalogData(double value[4])
{
    unsigned int data = uArm.mController.getServoAnalogData(value[0]);
    Serial.println(data);
}



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

void uArmComm::handleSerialData(char data)
{
    switch (mState)
    {
    case IDLE:
        if (data == '[')
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
        else if (data == '[')
        {

            cmdIndex = 0;
        }
        else if (data == ']')
        {

            cmdReceived[cmdIndex] = '\0';

            runCommand((char*)cmdReceived);
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


