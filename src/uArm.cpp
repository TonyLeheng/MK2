/**
  ******************************************************************************
  * @file	uArmClass.cpp
  * @author	David.Long	
  * @email	xiaokun.long@ufactory.cc
  * @date	2016-09-28
  * @license GNU
  * copyright(c) 2016 UFactory Team. All right reserved
  ******************************************************************************
  */

#include "uArm.h" 

#include "uArmComm.h"

uArmClass uArm;
extern uArmComm gComm;

uArmClass::uArmClass()
{
	mCurStep = -1;
	mTotalSteps = -1;
    mRecordAddr = 0;

    angleIndex[0] = -1;
    angleIndex[1] = -1;
    angleIndex[2] = -1;

    calState = INIT;

}

void uArmClass::initHardware()
{
	pinMode(BTN_D4, INPUT_PULLUP);        //special mode for calibration
	pinMode(BUZZER, OUTPUT);
	pinMode(LIMIT_SW, INPUT_PULLUP);
	pinMode(BTN_D7, INPUT_PULLUP);
	pinMode(PUMP_EN, OUTPUT);
    pinMode(GRIPPER, OUTPUT);

    #ifdef MKII
    pinMode(SYS_LED, OUTPUT);

    digitalWrite(PUMP_EN, HIGH);//keep the pump off
    #endif 

    #ifdef METAL
	pinMode(VALVE_EN, OUTPUT);
    #endif



}

void uArmClass::setup()
{

	initHardware();
	mController.init();

    mButtonD4.setPin(BTN_D4);
    mButtonD7.setPin(BTN_D7);

    gBuzzer.setPin(BUZZER);

    #ifdef MKII
    mLed.setPin(SYS_LED);
    #endif

    mCurStep = -1;
    mTotalSteps = -1;    

    test = 0;

}


void uArmClass::controllerRun()
{


	if (mCurStep >= 0 && mCurStep < mTotalSteps)
	{

		if((millis() - mStartTime) >= (mCurStep * mTimePerStep)) 
		{

            // ignore the point if cannot reach
			if (mController.limitRange(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]) != OUT_OF_RANGE_NO_SOLUTION)
			{
				//debugPrint("curStep:%d, %s, %s, %s", mCurStep, D(mPathX[mCurStep]), D(mPathY[mCurStep]), D(mPathZ[mCurStep]));
				if (mCurStep == (mTotalSteps - 1))
                {
                    double angles[3];
                    angles[0] = mController.getReverseServoAngle(0, mPathX[mCurStep]);
                    angles[1] = mController.getReverseServoAngle(1, mPathY[mCurStep]);
                    angles[2] = mController.getReverseServoAngle(2, mPathZ[mCurStep]);
                    debugPrint("curStep:%d, %s, %s, %s", mCurStep, D(angles[0]), D(angles[1]), D(angles[2]));
                    mController.writeServoAngle(angles[0], angles[1], angles[2]);
                    //mController.writeServoAngle(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]);
                }
                else
                {
                    mController.writeServoAngle(mPathX[mCurStep], mPathY[mCurStep], mPathZ[mCurStep]);
                }
			}
              
            mCurStep++;


           	if (mCurStep >= mTotalSteps)       
           	{
           		mCurStep = -1;
               
           	}  
		}	
	}
    else
    {
        mCurStep = -1;

    }
}

void uArmClass::recorderTick()
{
    //sys led function detec every 0.05s-----------------------------------------------------------------

    switch(mSysStatus)//every 0.125s per point
    {
    case SINGLE_PLAY_MODE:
        if(play() == false)
        {
                mSysStatus = NORMAL_MODE;
                mRecordAddr = 0;
        }
        break;

    case LOOP_PLAY_MODE:

        if(play() == false)
        {
            mRecordAddr = 0;
        }
        break;

    case LEARNING_MODE:
    case LEARNING_MODE_STOP:
        if(record() == false)
        {
                mSysStatus = NORMAL_MODE;
                mRecordAddr = 0;
           
                mController.attachAllServo();

        }
        break;

    default: 
        break;
    }

}

void uArmClass::systemRun()
{
//check the button4 status------------------------------------------------------------------------

    if (mButtonD4.clicked())
    {
    	//debugPrint("btnD4 down");
        mButtonD4.clearEvent();
        switch (mSysStatus)
        {
        case NORMAL_MODE:
        case NORMAL_BT_CONNECTED_MODE:
            mSysStatus = LEARNING_MODE;
            mRecordAddr = 0;//recording/playing address
            mController.detachAllServo();
            break;

        case LEARNING_MODE:
            //LEARNING_MODE_STOP is just used to notificate record() function to stop, once record() get it then change the sys_status to normal_mode
            mSysStatus = LEARNING_MODE_STOP;//do not detec if BT is connected here, will do it seperatly
            
            mController.pumpOff();
         
            break;

        default: break;
        }
    }

    

    //check the button7 status-------------------------------------------------------------------------
    if (mButtonD7.longPressed())
    {
        mButtonD7.clearEvent();
        switch(mSysStatus)
        {
        case NORMAL_MODE:
        case NORMAL_BT_CONNECTED_MODE:
            mRecordAddr = 0;
            mSysStatus = LOOP_PLAY_MODE;
            break;

        case SINGLE_PLAY_MODE:
        case LOOP_PLAY_MODE:
            break;

        case LEARNING_MODE: 
            break;
        }
    }
    else if (mButtonD7.clicked())
    {
        mButtonD7.clearEvent();
            //Serial.println("Test: whether BTN_D7 useful");
    	//debugPrint("btnD7 down");

        switch(mSysStatus)
        {
        case NORMAL_MODE:
        case NORMAL_BT_CONNECTED_MODE:
            mRecordAddr = 0;//recording/playing address
            mSysStatus = SINGLE_PLAY_MODE;  // or play just one time
            break;

        case SINGLE_PLAY_MODE:
        case LOOP_PLAY_MODE:
            mSysStatus = NORMAL_MODE;
            break;

        case LEARNING_MODE:
            if (digitalRead(PUMP_EN))
            {
                mController.pumpOff();
            }
            else
            {
                mController.pumpOn();
            }    
            break;
        }
    } 

	
}

void uArmClass::btDetect()
{
#ifdef MKII
    if ((mSysStatus == NORMAL_MODE) || (mSysStatus == NORMAL_BT_CONNECTED_MODE))
    {
        pinMode(BT_DETECT_PIN, INPUT);
        digitalWrite(BT_DETECT_PIN,HIGH);

        if (digitalRead(BT_DETECT_PIN) == HIGH)//do it here
        {
            mLed.on();
            mSysStatus = NORMAL_BT_CONNECTED_MODE;
        }
        else
        {
            mLed.off();
            mSysStatus = NORMAL_MODE;
        }

        pinMode(BT_DETECT_PIN, OUTPUT);
    }
#endif
}

void uArmClass::tickTaskRun()
{
    recorderTick();
    mButtonD7.tick();
    mButtonD4.tick();
#ifdef MKII
    mLed.tick();
    btDetect();
#endif    
}

void uArmClass::run()
{

#ifdef PRODUCTION
    gComm.run();
    servoCalibration();
#else
	
	controllerRun();
	systemRun();
#endif

    if(mTime50ms != millis() % TICK_INTERVAL)
    {
        mTime50ms = millis() % TICK_INTERVAL;
        if(mTime50ms == 0)
        {
            tickTaskRun();
        }
    }    

    if(mTime10ms != millis() % 50)
    {
        mTime10ms = millis() % 50;
        if(mTime10ms == 0)
        {
            Task10msRun();
        }
    }        
}

void uArmClass::stopMove()
{
	mCurStep = -1;
}

bool uArmClass::isMoving()
{
	return (mCurStep != -1);
}


bool uArmClass::play()
{

    unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper
    

    mRecorder.read(mRecordAddr, data, 5);
	debugPrint("mRecordAddr = %d, data=%d, %d, %d", mRecordAddr, data[0], data[1], data[2]);

    if(data[0] != 255)
    {
    	mController.writeServoAngle((double)data[2], (double)data[0], (double)data[1]);
        mController.writeServoAngle(SERVO_HAND_ROT_NUM, (double)data[3]);
        if (digitalRead(PUMP_EN) != data[4])
        {
            if (data[4])
            {
                mController.pumpOn();
            }
            else
            {
                mController.pumpOff();
            }   
        }
    }
    else
    {

        mController.pumpOff();
         
        return false;
    }

    mRecordAddr += 5;

    return true;
}

bool uArmClass::record()
{
	debugPrint("mRecordAddr = %d", mRecordAddr);

    if(mRecordAddr <= 65530)
    {
        unsigned char data[5]; // 0: L  1: R  2: Rotation 3: hand rotation 4:gripper
        if((mRecordAddr != 65530) && (mSysStatus != LEARNING_MODE_STOP))
        {
    		double rot, left, right;
    		//mController.updateAllServoAngle();
            mController.readServoAngles(rot, left, right);
			data[0] = (unsigned char)left;
			data[1] = (unsigned char)right;
            data[2] = (unsigned char)rot;
            data[3] = (unsigned char)mController.readServoAngle(SERVO_HAND_ROT_NUM);
            data[4] = (unsigned char)digitalRead(PUMP_EN);

            debugPrint("l=%d, r=%d, r= %d", data[0], data[1], data[2]);
        }
        else
        {
            data[0] = 255;//255 is the ending flag
            mRecorder.write(mRecordAddr, data, 5);

            return false;
        }

        mRecorder.write(mRecordAddr, data, 5);
        mRecordAddr += 5;

        return true;
    }
    else
    {
        return false;
    }

}

void uArmClass::interpolate(double startVal, double endVal, double *interpVals, int steps, byte easeType)
{

    startVal = startVal / 10.0;
    endVal = endVal / 10.0;

    double delta = endVal - startVal;
    for (byte i = 1; i <= steps; i++)
    {
        float t = (float)i / steps;
        //*(interp_vals+f) = 10.0*(start_val + (3 * delta) * (t * t) + (-2 * delta) * (t * t * t));
        *(interpVals + i - 1) = 10.0 * (startVal + t * t * delta * (3 + (-2) * t));
    }
}

unsigned char uArmClass::moveTo(double x, double y, double z, double speed)
{
	
	double angleRot = 0, angleLeft = 0, angleRight = 0;
	double curRot = 0, curLeft = 0, curRight = 0;
    double targetRot = 0;
    double targetLeft = 0;
    double targetRight = 0;
    double curX = 0;
    double curY = 0;
    double curZ = 0;
    int i = 0;
    int totalSteps = 0;
    unsigned int timePerStep;

    unsigned char status = 0;

    status = mController.coordianteToAngle(x, y, z, targetRot, targetLeft, targetRight);

    debugPrint("moveTo status: %d, angle: %s, %s, %s\n", status, D(targetRot), D(targetLeft), D(targetRight));

    if (status == OUT_OF_RANGE_NO_SOLUTION)
    {
    	return OUT_OF_RANGE_NO_SOLUTION;
    }

    // get current angles
    mController.getServoAngles(curRot, curLeft, curRight);
    // get current xyz
    mController.getCurrentXYZ(curX, curY, curZ);

    // calculate max steps
    totalSteps = max(abs(targetRot - curRot), abs(targetLeft - curLeft));
    totalSteps = max(totalSteps, abs(targetRight - curRight));

    if (totalSteps <= 0)
        return OUT_OF_RANGE_NO_SOLUTION;

    totalSteps = totalSteps < STEP_MAX ? totalSteps : STEP_MAX;

    // calculate step time
    double distance = sqrt((x-curX) * (x-curX) + (y-curY) * (y-curY) + (z-curZ) * (z-curZ));
    speed = constrain(speed, 100, 1000);
    timePerStep = distance / speed * 1000.0 / totalSteps;


    // keep timePerStep <= STEP_MAX_TIME
    if (timePerStep > STEP_MAX_TIME)
    {
        double ratio = double(timePerStep) / STEP_MAX_TIME;

        if (totalSteps * ratio < STEP_MAX)
        {
            totalSteps *= ratio;
            timePerStep = STEP_MAX_TIME;
        }
        else
        {
            totalSteps = STEP_MAX;
            timePerStep = STEP_MAX_TIME;
        }
    }


    totalSteps = totalSteps < STEP_MAX ? totalSteps : STEP_MAX;

    //debugPrint("totalSteps= %d\n", totalSteps);

    // trajectory planning
    interpolate(curX, x, mPathX, totalSteps, INTERP_EASE_INOUT_CUBIC);
    interpolate(curY, y, mPathY, totalSteps, INTERP_EASE_INOUT_CUBIC);
    interpolate(curZ, z, mPathZ, totalSteps, INTERP_EASE_INOUT_CUBIC);

    for (i = 0; i < totalSteps; i++)
    {
    	status = mController.coordianteToAngle(mPathX[i], mPathY[i], mPathZ[i], angleRot, angleLeft, angleRight);

    	if (status != IN_RANGE)
    	{
    		break;
    	}
    	else
    	{
    		mPathX[i] = angleRot;
    		mPathY[i] = angleLeft;
    		mPathZ[i] = angleRight;
    	}
    }

    if (i < totalSteps)
    {
    	interpolate(curRot, targetRot, mPathX, totalSteps, INTERP_EASE_INOUT_CUBIC);
    	interpolate(curLeft, targetLeft, mPathY, totalSteps, INTERP_EASE_INOUT_CUBIC);
    	interpolate(curRight, targetRight, mPathZ, totalSteps, INTERP_EASE_INOUT_CUBIC);    	
    }

    mPathX[totalSteps - 1] = targetRot;
    mPathY[totalSteps - 1] = targetLeft;
    mPathZ[totalSteps - 1] = targetRight;


#ifdef DEBUG 
    {
    	int i = 0;

    	for (i = 0; i < totalSteps; i++)
    	{
    		debugPrint("step%d, x=%s, y=%s, z=%s\n", i, D(mPathX[i]), D(mPathY[i]), D(mPathZ[i]));
    	}
    }
#endif   

    mTimePerStep = timePerStep;
    mTotalSteps = totalSteps;
    mCurStep = 0;
    mStartTime = millis();

    return IN_RANGE;
}

#ifdef MKII
bool uArmClass::isPowerPlugIn()
{
    if (analogRead(POWER_DETECT) > 400)
        return true;
    else
        return false;
}
#endif 

void uArmClass::Task10msRun()
{
    switch(calState)
    {
    case INIT:
        break;

    case START:
    debugPrint("START");
        waitReady = true;
        Serial.println("[C]"); // clear

        
        
        calState = READY;   

        break;

    case READY:
        if (!waitReady)
        {
            debugPrint("READY");
            angleIndex[0] = -1;
            angleIndex[1] = -1;
            angleIndex[2] = -1;   

            curAngle[0] = 0;
            curAngle[1] = 0;
            curAngle[2] = 0;  
                       
            lastAngle[0] = 0;
            lastAngle[1] = 0;
            lastAngle[2] = 0;

            mController.writeServoAngle(0, 0, false);
            mController.writeServoAngle(1, 0, false);
            mController.writeServoAngle(2, 0, false);

            delay(500);

            m10msCount = 0; 
            calState = PROCESS;
        }
        break;

    case PROCESS:
        {
            if ((m10msCount % 3) == 0)
            {
                int servoNum = m10msCount / 3;

                if (curAngle[servoNum] > 0 && curAngle[servoNum] <= 180 && lastAngle[servoNum] != curAngle[servoNum])
                {
                    lastAngle[servoNum] = curAngle[servoNum];
                    //String str = String("D" + servoNum);//  + String("V" + curAngle[servoNum]);
                    Serial.print("[D");
                    Serial.print(servoNum); // read real angle
                    Serial.print("V");
                    Serial.print(curAngle[servoNum]);
                    Serial.println("]");
                }
            }
            else if ((m10msCount % 3) == 1)
            {
                int servoNum = m10msCount / 3;

                if (curAngle[servoNum] == 0 )
                {
                    curAngle[servoNum] = 1;

                    

                    mController.writeServoAngle(servoNum, curAngle[servoNum], false);
               
                }    
                else if (angleIndex[servoNum] == curAngle[servoNum])
                {
                    // record the data
                    // servoNum, angleIndex[servoNum], angleValue[servoNum]
                    unsigned char data[2];
                    unsigned int startAddr = 0;

                    unsigned int offset = 0;

                    if (angleIndex[servoNum] == 2 && angleValue[servoNum] < 10)
                    {
                        //error, restart
                        calState = READY;
                    }


                    debugPrint("write next");
                    switch (servoNum)
                    {
                    case 0:
                        startAddr = ROT_SERVO_ADDRESS;
                        break;
                    case 1:
                        startAddr = LEFT_SERVO_ADDRESS;
                        break;
                    case 2:
                        startAddr = RIGHT_SERVO_ADDRESS;
                        break;
                    }

           
                    data[0] = (angleValue[servoNum] & 0xff00) >> 8;
                    data[1] = angleValue[servoNum] & 0xff;

                    offset = (angleIndex[servoNum] - 1 ) * 2;

                    iic_writebuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, startAddr + offset, 2);

                    curAngle[servoNum]++;

                    if (curAngle[servoNum] <= 180)
                    {
                        mController.writeServoAngle(servoNum, curAngle[servoNum], false);
                    }
                }         
            }

            m10msCount++;

            if (m10msCount >= 9)
                m10msCount = 0;

            if (curAngle[0] > 180 && curAngle[1] > 180 && curAngle[2] > 180)
            {
                gBuzzer.buzz(4000, 500);   
                m10msCount = 0;
                calState = GETADC;  

                maxAngle[0] = angleValue[0] / 10 - 1;
                maxAngle[1] = angleValue[1] / 10 - 1;
                maxAngle[2] = angleValue[2] / 10 - 1;



                // write to 500 addr
                unsigned char data[2];
                unsigned int offset = 0;
                for (int k = 0; k < 3; k++)
                {
                    data[0] = (maxAngle[k] & 0xff00) >> 8;
                    data[1] = maxAngle[k] & 0xff;

                    offset = (4 + k) * 1024 + 500;
                    delay(10);
                    iic_writebuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, offset, 2);
                    
                }

                curAngle[0] = 0;
                curAngle[1] = 0;
                curAngle[2] = 0;

                for (int j = 0; j < 3; j++)
                {
                    mController.writeServoAngle(j, curAngle[0], true);

                }    

                delay(1000);
            }

            break; 

        case GETADC:
            m10msCount++;
            if (m10msCount >= 5)
            {
                m10msCount = 0;
                
                for (int i = 0; i < 3; i++)
                {
                    if (curAngle[i] <= maxAngle[i])
                    {
                        getAdcTable(i, curAngle[i]);
                        
                    }

                    curAngle[i]++;
                }

                if (curAngle[0] > maxAngle[0] && curAngle[1] > maxAngle[1] && curAngle[2] > maxAngle[2])
                {
                    curAngle[0] = maxAngle[0];
                    curAngle[1] = maxAngle[1];
                    curAngle[2] = maxAngle[2];


                    calState = GETREADC;  




                    for (int j = 0; j < 3; j++)
                    {
                        mController.writeServoAngle(j, curAngle[0], true);
                    }                    
                }

            }
            break;

        case GETREADC:
            m10msCount++;
            if (m10msCount >= 5)
            {
                m10msCount = 0;

                for (int i = 0; i < 3; i++)
                {
                    if (curAngle[i] >= 0)
                    {
                        getReAdcTable(i, curAngle[i]);
                        curAngle[i]--;
                    }
                }
                
                if (curAngle[0] < 0 && curAngle[1] < 0 && curAngle[2] < 0)
                {
                    // done  
                     gBuzzer.buzz(4000, 500);   
                     calState = INIT;             
                }

            }        
            break;

        }
        
    }
}

void uArmClass::getAdcTable(int servoNum, int angle)
{
    unsigned char data[2];

    Serial.print("getAdcTable:");
    Serial.print(servoNum);
    Serial.print(":");
    Serial.println(angle);
    // read adc 
    unsigned int adcData = mController.getServoAnalogData(servoNum);
    Serial.print("adc:");
    Serial.println(adcData);


// write to E2PROM
    data[0] = (adcData & 0xff00) >> 8;
    data[1] = adcData & 0xff;

    iic_writebuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, (4 + servoNum) * 1024 + (angle)*2, 2);

// write next angle
    angle++;

    if (angle <= maxAngle[servoNum])
    {
        mController.writeServoAngle(servoNum, angle, true);
    }

    
}

void uArmClass::getReAdcTable(int servoNum, int angle)
{
    unsigned char data[2];

    Serial.println("getReAdcTable");
  
    // read adc 
    unsigned int adcData = mController.getServoAnalogData(servoNum);
    Serial.print("adc:");
    Serial.println(adcData);

// get angle from adc table
    int angleFromTable = mController.analogToAngle(servoNum, adcData);



// get the diff of angles
    int diff = angleFromTable - angle;

    data[0] = (diff & 0xff00) >> 8;
    data[1] = diff & 0xff;

// write to E2PROM
   iic_writebuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, (4 + servoNum) * 1024 + 512 + (angle)*2, 2);


// write next angle
    angle--;

    if (angle >= 0)
    {
        mController.writeServoAngle(servoNum, angle, true);
    }       
    
}

#ifdef PRODUCTION
void uArmClass::updateServoAngleData(int servoNum, int index, int data)
{
    debugPrint("updateServoAngleData");
    angleIndex[servoNum] = index;
    angleValue[servoNum] = data;

    debugPrint("%d, %d, %d", servoNum, angleIndex[servoNum], angleValue[servoNum]);
}

void uArmClass::servoCalibration()
{
    if (mButtonD4.clicked())
    {
        mButtonD4.clearEvent();
        if (calState == INIT)
        {
            debugPrint("click");
            calState = START;
        }
        else
        {
            debugPrint("click2");
            calState = INIT;
        }
    }

    if (mButtonD7.clicked())
    {
        mButtonD7.clearEvent();
        
        // double angle = test;
        // Serial.println(test);
        // mController.readServoCalibrationData(ROT_SERVO_ADDRESS, angle);
        // test++;

        unsigned char data[2];
        iic_readbuf(data, EXTERNAL_EEPROM_SYS_ADDRESS, 4 * 1024 + test*2, 2);
        unsigned int max = (data[0] << 8) + data[1];   
        Serial.println(test);
        Serial.println(max);    
        test++; 
        
    }
}
#endif