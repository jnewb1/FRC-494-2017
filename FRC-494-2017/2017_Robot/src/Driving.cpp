//
//
// BS File
//

//#include <iostream.h>
#include "WPILib.h"
#include "Driving.h"
#include "motor.h"
#include "iencoder.h"
#include "cal.h"
#include "motion.h"
#include "joystick.h"
#include "itimer.h"
#include "aton.h"
#include <math.h>
#include <cmath>
//#include "DriverStationLCD.h"


void ShootBalls2();
void MoveToolbarToPOS(float position);
void MoveTurretToPOS(float position);
void MoveHopperToPOS(float POS);
void SetShooterToRPM(float rpm);
void Set_BallIntake(float speed);
void Set_GearIntake(float speed);
void PegAim();
void stopPegAim();
void Set_Hopper(float speed);
void Set_Winch1(float speed);
void Set_Winch2(float speed);
void Set_BallIntake     (float speed);
void Set_GearIntakeRight(float speed);
void Set_GearIntakeLeft (float speed);
void Set_Winch1         (float speed);
void Set_Winch2         (float speed);
void Set_ToolBar        (float speed);
void Set_Shooter        (float speed);
void Set_Hopper         (float speed);
void Set_GearIntake		(float speed);
void Set_Winch          (float speed);
void Set_Turret         (float speed);
void setAntiBackDrive   (float pos);
float Get_ShooterSetPoint();
float Get_TurretPOS();
float Get_ShooterSpeed  ();
float Get_HopperPOS();
extern float currentHopperPOS;

void MoveToolBarToGearPickup();
void MoveToolBarToGearDropoff();

void ManualClimbUp();
void ManualClimbDown();
void ManualClimbStop();
void StartLoadGear();
void StopLoadGear();
void StartLoadBall();
void StopLoadBall();
void MoveToolBarUp();
void MoveToolBarDown();
void StopToolBar();
void StopLoadGear();
void StartDropGear();
void StopDropGear();



void MoveTurretTo0(unsigned char);
void MoveTurretTo90(unsigned char);
void MoveTurretTo180(unsigned char);

void GoalAim();
void stopGoalAim();
void ShootBalls();
void DontShootBalls();
void StopShootingBalls();
void ShootingBalls();
void ManualShooter();

void switchCam();
void SetHopperToRPM(float RPM);

bool ManualBallIntake = false;

extern float sBlockTime;

extern int gTimeOut;

extern float xDrift;
extern int   xDriftCount;

extern double beginAtonTime;

void SetSolenoid(int num, int state);

int blockToroAuto = 0;

float crossHairX = 80;

int   stopAllDriveTrain = 0;

int   forceSpeed = 0;

int timeToSteeingOff = 0;

void SaveGlobalData();

int blockDriving = 0;

int steeringOff = 0;

void sonarShot(int shot);


void Limit127(float &value)
{
	if (value > 127) value = 127;
	if (value < -127) value = -127;
}


int drivingOff = 0;

int testFollow = 0;

static float holdLeft = 0;
static float holdRight = 0;

void drivingService(void)
{
	//printf("hhh\n");
	if (!newDrivingInfo) return;

	// setRightDriveTrain(128+128*joyY [2],0);
	 //setLeftDriveTrain (128+128*joyY2[2],0);

	 //return;


	newDrivingInfo = 0;

	// if ( (fabs(joyY[2])+fabs(joyY[3]) +fabs(joyX[2]) +fabs(joyX[3])) >0.5)

	if (joyY[2]<-0.3 || joyY[3]<-0.3 || fabs(joyX[2])>0.3 || fabs(joyX[3])>0.3) //FailSafe
	{
		forceSpeed = 0;
		atonActive = 0;
		stopAllDriveTrain = 0;
		blockDriving = 0;
	}



	if (forceSpeed || drivingOff) return;

	if (stopAllDriveTrain)
	{

		setRightDriveTrain(128, 0);
		setLeftDriveTrain(128, 0);


		return;
	}

	if (blockDriving || atonActive) return;


	if (DriveModeFlag == 0) //no power to motors (unless forced)
	{
		setRightDriveTrain(0, 0);
		setLeftDriveTrain(0, 0);
	}


	if (DriveModeFlag == 1) //Joystick Driving (High Gear)
	{
		float rateR = 0;
		float rateL = 0;

		if (holdStraight)
		{
			rateR = (joyX[2] + joyX[3]) * 126;
			rateL = (joyX[2] + joyX[3]) * 126;
		}
		else
		{
			rateR = (joyX[2] + joyX[3]) * 126; //+joyX[1]/2
			rateL = (joyX[2] + joyX[3]) * 126; //+joyX[1]/2
		}

		// float rateR= (joyX[2])*126;
		// float rateL= (joyX[2])*126;


		float y = joyY[2] + joyY[3]; //+joyY[1]);

		if (holdStraight && gyroShift == 0) y += joyY[1];

		if (inAt) y = 0;

		rateR = y * 128 + (1.0 + fabs(y))*rateR;
		rateL = y * 128 - (1.0 + fabs(y))*rateL;

		setRightDriveTrain(128 + rateR, 0);
		setLeftDriveTrain(128 + rateL, 0);
	}

	if (DriveModeFlag == 2) //Joystick Driving (Low Gear)
	{
		float rateR = 0;
		float rateL = 0;

		if (holdStraight)
		{
			rateR = (joyX[2]) * 126;
			rateL = (joyX[2]) * 126;
		}
		else
		{
			rateR = (joyX[2]) * 126; //+joyX[1]/2
			rateL = (joyX[2]) * 126; //+joyX[1]/2
		}
		//  float rateR= (joyX[2])*126;
		//  float rateL= (joyX[2])*126;

		float y = joyY[2] + joyY[3];//+joyY[1];

		if (holdStraight) y += joyY[1];

		if (inAt) y = 0;

		rateR = y * 128 + (1.0 + fabs(y))*rateR;
		rateL = y * 128 - (1.0 + fabs(y))*rateL;

		setRightDriveTrain(128 + rateR, 1);
		setLeftDriveTrain(128 + rateL, 1);
	}



	if (DriveModeFlag == 3) //Hold Position (using current shift mode)
	{

		float diff = loc[aRightDist] - holdRight;

		float speedR = -diff*holdGainR;

		if (speedR > 0.3) speedR = 0.3;
		if (speedR < -0.3) speedR = -0.3;

		int rightRate = (int)(128 + 128 * speedR);

		diff = loc[aLeftDist] - holdLeft;

		float speedL = -diff*holdGain;

		if (speedL > 0.3) speedL = 0.3;
		if (speedL < -0.3) speedL = -0.3;

		int leftRate = (int)(128 + 128 * speedL);

		if (holdGainR == 0) rightRate = leftRate;
		if (holdGain == 0) leftRate = rightRate;

		if (holdGainR || holdGain)
		{
			setRightDriveTrain(rightRate, -1);
			setLeftDriveTrain(leftRate, -1);
		}

	}
	else
	{
		holdRight = loc[aRightDist];
		holdLeft = loc[aLeftDist];
	}


}

void resetTurretCmd()
{
	//resetEncoder(aAzimuth);
}

//extern DriverStationLCD *dsLCD;

//blueTeam=0;redTeam=1;

void DisplayDsInfo()
{

	char mes[100];

	char teamColor[10] = "Blue"; if (redTeam) strcpy(teamColor, "Red ");

	char atonMes[30] = "No Motion! ";

	if (atonNum == 1) strcpy(atonMes, " ONE       ");
	if (atonNum == 2) strcpy(atonMes, " TWO       ");
	if (atonNum == 3) strcpy(atonMes, " THREE     ");
	if (atonNum == 4) strcpy(atonMes, " FOUR      ");
	if (atonNum == 5) strcpy(atonMes, " Special 1 ");
	if (atonNum == 6) strcpy(atonMes, " Special 2 ");

	sprintf(mes, "%s %d: %s ", teamColor, atonNum, atonMes);

	//dsLCD->Printf(DriverStationLCD::kMain_Line6, 1, mes);

	//dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "");
	//dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "");
	//dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "");
	//dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "");

	sprintf(mes, "Delay: %f ", atonDelayTime);

	//dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, mes);

	//dsLCD->UpdateLCD();

}

void NextAton()
{

	if (!inDisable) return;

//	atonNum++; if (atonNum > 6) atonNum = 0;

	SaveGlobalData();

	DisplayDsInfo();

} //NextAton


void NextStation()
{
	stationNum++; if (stationNum > 1) stationNum = 0;

	if (stationNum == 0) { blueTeam = 0; redTeam = 1; }
	else { redTeam = 0; blueTeam = 1; }

	SaveGlobalData();

	DisplayDsInfo();

} //NextStation


void reduceAtonDelay() { atonDelayTime -= 0.5; if (atonDelayTime < 0)atonDelayTime = 0; SaveGlobalData(); DisplayDsInfo(); }
void increaseAtonDelay()
{
	if (!inDisable) return;

	atonDelayTime += 0.5;

	if (atonDelayTime > 2)atonDelayTime = 0;

	SaveGlobalData();

	DisplayDsInfo();
}

//void fastVidOn(){fastVideo=1;}
//void fastVidOff(){fastVideo=0;}

extern int kickVideoServer;

void ReStartCamera();

void kickVideo() {

	//ReStartCamera();

	//kickVideoServer=1;

}

void nextAtonDealy() {

	atonDelayTime += 0.5;
	if (atonDelayTime > 5)atonDelayTime = 0;
	SaveGlobalData();
	DisplayDsInfo();

}
























void setStopDropAngle(float angle);
void setShifterAngle(float angle);

void testa() { setShifterAngle(0); };
void testb() { setShifterAngle(180); };

void TrackStart()
{
	//forceSteer=1;
}

//static int    TargetZ=0;
//static int    TargetX=0;
//static int    TargetW=0;
//static double TargetA=0;

//static int TargetCount=0;

void TrackService()
{
	//forceSteerF=0.5;
	//forceSteerB=0.5;

   // int    tarX=0;
   // int    tarZ=0;
   // int    tarW=0;
   // double tarA=0;

   // if (TargetCount && mTargetX==0)
	{
		//	 TargetCount--;

		//	 tarX=TargetX;
		////	 tarZ=TargetZ;
		//	 tarW=TargetW;
		//	 tarA=TargetA;
	}
	// else
	 //{
	//	 tarX=mTargetX;
	//	 tarZ=mTargetZ;
	//	 tarW=mTargetW;
	//	 tarA=mTargetA;

		// TargetX=tarX;
		// TargetZ=tarZ;
		// TargetW=tarW;
		// TargetA=tarA;
	 //}

	// if (mTargetX) TargetCount=20;

	// if (tarX)
	// {
	 // float ang=-tarA/400.0;

	   //if (tarX>80) ang=-ang;

	   //forceSteerF=0.5+ang;

	  // if (forceSteerF<0.25) forceSteerF=0.25;
	   //if (forceSteerF>0.75) forceSteerB=0.75;
	//}

}


void TrackEnd()
{
	//forceSteer=0;

}

//------------------------------------

float spinCener = 80;
float spinGain = 5;
float gyroSpinScale = 1;
float spinMin = 13;
float targetLockCount = 5;
float autoKickCount = 5;
float iGainSpinKick = 0.1;
float autoKickRange = 2;

double spinAngle = 0;

float iSpinGain = 0;

void SpinStart()
{
	spinAngle = 0;

	forceSpeed = 1;
	//spinFlag  =1;
	//forceSteer=1;

	iSpinGain = 1;
}

static int targetCount = 0;

void SpinService()
{
	//forceSteerF=0.5;
	//forceSteerB=0.5;

	if (spinAngle == 0)
	{
		//	if (mTargetX) targetCount++; else targetCount=0;

		if (targetCount < targetLockCount) return;

		//  spinAngle=getGyroAngle()*gyroSpinScale+mTargetA;

		targetCount = 0;

		forceSpeed = 1;
		//spinFlag  =1;
		//forceSteer=1;

		iSpinGain = 1;

		return;
	}

	float spinError = getGyroAngle()*gyroSpinScale - spinAngle;

	float rate = spinError*spinGain*iSpinGain;

	if (rate < 0) rate -= spinMin;
	if (rate > 0) rate += spinMin;

	if (rate > 64) rate = 64;
	if (rate < -64) rate = -64;

	setRightDriveTrain(128 + rate, 0);
	setLeftDriveTrain(128 - rate, 0);

	if (fabs(spinError) > autoKickRange)
	{
		iSpinGain += iGainSpinKick;
	}
	else
	{
		iSpinGain = 1;
	}
	if (iSpinGain > 20.0) iSpinGain = 20;

}


void SpinEnd()
{
	forceSpeed = 0;
	//spinFlag=0;
	//forceSteer=0;

	spinAngle = 0;
}

//------------------------------

void spinKickFinished(uc i)
{
	SpinEnd();
	blockGyroReset = 0;
	atonActive = 0;
}

void spinKickService(uc i)
{
	SpinService();

	if (spinAngle)
	{
		float spinError = getGyroAngle()*gyroSpinScale - spinAngle;

		if (fabs(spinError) < autoKickRange)

			tripTimer(i);
	}

}

void spinKickAtonXX()
{
	//if (!mTargetX) return;

//	atonActive=500;blockGyroReset=-500;spinAngle=0;targetCount=0;

//	startTimer(15,300,spinKickService,spinKickFinished);

//	SpinStart();
}

void setPwmFg(int L, float val);

//--- hood Control ----

float hoodDest = 0;
float hoodVel = 0;


void moveHoodService(uc i);
void moveHoodServiceN(uc i);
void moveHoodFinished(uc i);

void(*moveHoodAtFinish)() = 0;

void moveHoodFinishedd();

void moveHood(float dest, float vel, void(*doAtFinish)())
{
	/*
	moveUpDownAtFinish=doAtFinish;

	upDownDest=dest;

	//tprintf("  moveUpDown %6.3f -> %6.3f\n",loc[aUpDown],dest);

	float dif=fabs(dest-loc[aUpDown]);

	if (dif<2) {safe(moveUpDownFinishedd); return; }

	if (dest<loc[aUpDown])
	{
	   eLevel=-2;

	   upDownVel =fabs(elevDownRate);
	   startTimer(aUpDown,400,moveUpDownServiceN,moveUpDownFinished);
	}
	else
	{
		eLevel=-3;

		upDownVel =fabs(elevUpRate);
		upDownVel =vel;

		startTimer(aUpDown,400,moveUpDownService,moveUpDownFinished);
	}
	*/
}

void moveHoodService(uc i)
{
	/*
	float dif=upDownDest-loc[aUpDown];

	if (dif<0) dif=10;

	float velScale=1.0;

	if (dif<10) velScale=0.5+dif/10.0;

	float diff=fabs(loc[aUpDown]-upDownDest);

	if (diff<1.5) { tripTimer(aUpDown); return; }

	//if (loc[aUpDown]>=upDownDest) { tripTimer(aUpDown); return; }

	setPwmFg(pUpDown,upDownVel*velScale);

	blockUpDownUser=1000; //one second
	*/
}

void moveHoodServiceN(uc i)
{
	/*
	float dif=loc[aUpDown]-upDownDest;

	if (dif<0) dif=10;

	float velScale=1.0;

	if (dif<10) velScale=0.5+dif/20.0;

	float diff=fabs(loc[aUpDown]-upDownDest);

	if (diff<1.5 || UpDownLimit) { tripTimer(aUpDown); return; }

	//if (loc[aUpDown]<=upDownDest) {tripTimer(aUpDown); return;}

	setPwmFg(pUpDown,-upDownVel*velScale);

	blockUpDownUser=1000; //one second
	*/
}

void moveHoodFinishedd()
{
	/*
	blockUpDownUser=0;
	setPwmFg(pUpDown,0);

	eLevel=-1;

	safe(moveUpDownAtFinish);
*/
}

void moveHoodFinished(uc i)
{
	moveHoodFinishedd();
}

//--- Azimth Control ----

float azimthDest = 0;
float azimthVel = 0;


void moveAzimthService(uc i);
void moveAzimthServiceN(uc i);
void moveAzimthFinished(uc i);

void(*moveAzimthAtFinish)() = 0;


void moveAzimth(float dest, float vel, void(*doAtFinish)())
{
	/*
		safe(doAtFinish);

		return;

		if (dest==-1) { tprintf("  moveAzimth [no Move]\n");safe(doAtFinish);return;}

		float dif=fabs(dest-loc[aAzimuth]);

		if (dif<3) {tprintf("  moveAzimth [no Move]\n");safe(doAtFinish); return;}

		moveAzimthAtFinish=doAtFinish;

		if (vel<0) vel=-vel;

		azimthDest=dest;
		azimthVel =vel;

		tprintf("  moveAzimth %6.3f -> %6.3f\n",loc[aAzimuth],dest);



		if (dest<loc[aAzimuth])
		{
		startTimer(aAzimuth,300,moveAzimthServiceN,moveAzimthFinished);
		}
		else
		{
		startTimer(aAzimuth,300,moveAzimthService,moveAzimthFinished);
		}
		*/
}

void moveAzimthService(uc i)
{
	/*
	float dif=azimthDest-loc[aAzimuth];

	float velScale=1.0;

	if (dif<10) velScale=0.5+dif/20.0; //0.1+dif*0.045;

	if (loc[aAzimuth]>azimthDest) { tripTimer(aAzimuth); return; }

	//printf(" movePos %6.3f \n",azimthVel*velScale);

	setPwmFg(pAzimuth,-azimthVel*velScale);

	blockAzimthUser=1000; //one second
	*/
}

void moveAzimthServiceN(uc i)
{
	/*
	float dif=loc[aAzimuth]-azimthDest;

	float velScale=1.0;

	if (dif<10) velScale=0.5+dif/20.0; //0.1+dif*0.045;

	if (loc[aAzimuth]<azimthDest) {tripTimer(aAzimuth); return;}

	//printf(" moveNeg %6.3f \n",azimthVel*velScale);

	setPwmFg(pAzimuth,azimthVel*velScale);

	blockAzimthUser=1000; //one second
	*/
}

void moveAzimthFinished(uc i)
{
	/*
	blockAzimthUser=0;
	setPwmFg(pAzimuth,0);

	safe(moveAzimthAtFinish);
	*/
}






//bool IsDisabled(){return false;}







bool isDis(char *str) { tprintf(str); if (IsDisabled()) { printf(" is Disabled\n"); return true; } else { printf("\n"); return false; } }

#define ifLive(A) if (isDis(A)) return;

#define ifLivv(A) printf("\n");if (isDis(A)) return;









void StartTracking()
{
	//tprintf("StartTracking\n");

	//fastVideo=0;

}
void StopTracking()
{
	//tprintf("StopTracking\n");

	//fastVideo=1;
}




//const int xboxA=1;
//const int xboxB=2;
//const int xboxX=3;
//const int xboxY=4;
//const int xboxLS=5;
//const int xboxRS=6;
//const int xboxBack=7;
//const int xboxStart=8;
//const int xboxLeftStickClick=9;
//const int xboxRightStickClick=10;











extern float shiftRight;
extern float shiftLeft;

void RightShiftFast()
{
	shiftRight = 1;
}

void RightShiftSlow()
{
	shiftRight = 0;
}

void LeftShiftFast()
{
	shiftLeft = 1;
}

void LeftShiftSlow()
{
	shiftLeft = 0;
}



void toggleRightShift()
{
	if (shiftRight != 0) shiftRight = 0; else shiftRight = 1;
}

void toggleLeftShift()
{
	if (shiftLeft != 0) shiftLeft = 0; else shiftLeft = 1;
}

void force(int motor, float value)
{
	forceValue[motor] = value;
	forceCount[motor] = 100;
}

void forceRight(float value)
{
	force(pwmRightMotor1, value);
	force(pwmRightMotor2, value);
}

void forceLeft(float value)
{
	force(pwmLeftMotor1, value);
	force(pwmLeftMotor2, value);
}

float holdLoc[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

void holdWheel(int index)
{
	/*
	if (index<1 || index>2) return;

	float diff=loc[index]-holdLoc[index];

	float speed=-diff/1000.0;

	if (speed> 0.3) speed= 0.3;
	if (speed<-0.3) speed=-0.3;

	if (index==aRightWheel) forceRight(speed);
	if (index==aLeftWheel ) forceLeft (speed);
	*/

}

void holdRobotStart()
{
	holdLoc[1] = loc[1];
	holdLoc[2] = loc[2];
}

void holdRobotService()
{
	//holdWheel(aRightWheel);
	//holdWheel(aLeftWheel );
}

void holdRobotStop()
{
	forceRight(0);
	forceLeft(0);
}



void moveRobotStart()
{
	holdLoc[1] = loc[1];
	holdLoc[2] = loc[2];
}

void moveRobotService()
{
	//holdWheel(aRightWheel);
	//holdWheel(aLeftWheel );

	//holdLoc[1]+=10;
	//holdLoc[2]+=10;

}

void moveRobotStop()
{
	forceRight(0);
	forceLeft(0);
}

void ArmDown()
{
	armFlag = 1;
}
void ArmStop()
{
	armFlag = 0;

}

void ArmUp()
{

	armFlag = 2;
}

extern int shiftDelay;

void tglShiftModeCmd()
{
	if (ShiftModeFlag) ShiftModeFlag = 0; else ShiftModeFlag = 1;

	sendBack(57, ShiftModeFlag);

	shiftDelay = 200;
}

void fullStop();

//float fastVideoHold=0;

int spitBallsCount = 0;

int inFireTheBall = 0;


/*
void FireTheBallCmd()  //Send Ball and Disable Turret
{
	//fastVideoHold=fastVideo;

	inFireTheBall = 1;

	if (spitBallsCount) { fullStop(); return; }

	rollerDeadZone = 4; sendBack(145, rollerDeadZone);

	//	fastVideo=1;    sendBack(145,fastVideo);
}

void FireTheBallCmdService()
{

	if (spitBallsCount) { fullStop(); return; }

}

void FireTheBallCmdEnd() //Re-Enable Turret
{
	//	fastVideo=fastVideoHold; sendBack(145,fastVideo);

	if (spitBallsCount) { fullStop(); }

	inFireTheBall = 0;
}


int blockSpit = 0;

void spitBallsBeg()
{
	spitBallsCount = 0;

	blockSpit = 0;

	if (inFireTheBall) { blockSpit = 1; fullStop(); return; }
}

void spitBallsService()
{
	if (inFireTheBall || blockSpit) { blockSpit = 1; return; }

	spitBallsCount++;

	if (spitBallsCount > 50)
	{
		rollerDeadZone = 2;

		PickupRollerFlag = 2;
	}
}
void CollectBallsTgl()
{
	spitBallsCount = 0;

	if (inFireTheBall || blockSpit) { blockSpit = 0; fullStop(); return; }

	if (rollerDeadZone)
	{
		rollerDeadZone = 0;
		PickupRollerFlag = 0;
	}
	else

		rollerDeadZone = 3;
}
*/
void spitBallsEnd()
{
	spitBallsCount = 0;
}



int manualSetOn = 0;

float msHoldSpeed = 0;
float msHoldHood = 0;
float msHoldTrack = 0;
float msShooterModeFlag = 0;

void saveSettings()
{

	msHoldSpeed = wantedShooterRPM;
	msHoldHood = wantedArmAngle;
	//msHoldTrack=fastVideo;
	msShooterModeFlag = ShooterModeFlag;

}

void loadSettings()
{

	wantedShooterRPM = msHoldSpeed;
	wantedArmAngle = msHoldHood;
	//fastVideo       =msHoldTrack;
	ShooterModeFlag = msShooterModeFlag;

}

void sendSetting()
{
	sendBack(179, wantedArmAngle);
	sendBack(63, wantedShooterRPM);
	//  sendBack(132,fastVideo       );
	sendBack(188, ShooterModeFlag);


}


void manualSetA()
{


}

void manualSetB()
{
	tprintf("manualSetB\n");
	//	 if (manualSetOn) return;

	//	 manualSetOn=1;

	//	 saveSettings();

		// if (B_ShotAngBeg) wantedShooterRPM =B_ShotAngBeg;

	static int tgl = 0;

	if (tgl == 0) if (B_ShotAngEnd) wantedArmAngle = B_ShotAngEnd;
	if (tgl == 1) if (B_ShotSpeed) wantedArmAngle = B_ShotSpeed;

	tgl++; if (tgl > 1) tgl = 0;
	/*
	if (B_ShotSpeed==1) fastVideo=0;

	if (B_ShotSpeed==3) armFlag=3;
	if (B_ShotSpeed==4) armFlag=4;
	if (B_ShotSpeed==5) armFlag=5;
	if (B_ShotSpeed==6) armFlag=6;
	   */

	ShooterModeFlag = 0;

	//	 sendSetting();

}

void manualSetX()
{
	tprintf("manualSetX\n");

	//	 if (manualSetOn) return;
	//
	//	 manualSetOn=1;

	//	 saveSettings();

	if (X_ShotAngBeg) wantedShooterRPM = X_ShotAngBeg;
	if (X_ShotAngEnd) wantedArmAngle = X_ShotAngEnd;
	if (X_ShotSpeed) armFlag = X_ShotSpeed;
	/*
	if (X_ShotSpeed==1) fastVideo=0;

	if (X_ShotSpeed==3) armFlag=3;
	if (X_ShotSpeed==4) armFlag=4;
	if (X_ShotSpeed==5) armFlag=5;
	if (X_ShotSpeed==6) armFlag=6;
	   */

	ShooterModeFlag = 0;

	//	 sendSetting();

}

void manualSetY()
{
	tprintf("manualSetY\n");
	//if (manualSetOn) return;

	 //  manualSetOn=1;

	//   saveSettings();

	if (Y_ShotAngBeg)  wantedShooterRPM = Y_ShotAngBeg;
	if (Y_ShotAngEnd)  wantedArmAngle = Y_ShotAngEnd;
	if (Y_ShotSpeed)  armFlag = Y_ShotSpeed;
	/*
	if (Y_ShotSpeed==1) fastVideo=0;

	if (Y_ShotSpeed==3) armFlag=3;
	if (Y_ShotSpeed==4) armFlag=4;
	if (Y_ShotSpeed==5) armFlag=5;
	if (Y_ShotSpeed==6) armFlag=6;
	 */

	ShooterModeFlag = 0;

	//   sendSetting();


}


void manualSetService()
{


}

void manualSetEnd()
{

	if (!manualSetOn) return;

	manualSetOn = 0;

	loadSettings();

	sendSetting();

}

void startAtonSelected();


void toggleAutoTracking()
{
	//if (ShooterModeFlag==0)
	//{
	//	ShooterModeFlag=10;sendBack(188,ShooterModeFlag);
	//}
	//else
	//{
	//	ShooterModeFlag=0;sendBack(188,ShooterModeFlag);

	//    fastVideo=1;      sendBack(132,fastVideo);
	//}
}

void toggleTurretTracking()
{
	//if (fastVideo==0)
	//{
	//	ShooterModeFlag=0;sendBack(188,ShooterModeFlag);
		//fastVideo=1;      sendBack(132,fastVideo);

	//}
	//else
	//{
	//	ShooterModeFlag=0;sendBack(188,ShooterModeFlag);

	  //  fastVideo=0;      sendBack(132,fastVideo);
	//}
}

void fullStop();

extern int driverSelect;

void driverSelectCmd()
{
	//if (driverSelect==1) driverSelect=2; else driverSelect=1;

}



void tglAppatureFlagTop()
{
	switch ((int)AppStateTop)
	{
	case 0: AppStateTop = 1; break;
	case 1: AppStateTop = 2; break;
	default: AppStateTop = 0; break;
	}
}
void tglAppatureFlagMid()
{
	switch ((int)AppStateMid)
	{
	case  0: AppStateMid = 1; break;
	case  1: AppStateMid = 2; break;
	default: AppStateMid = 0; break;
	}
}
void tglAppatureFlagBot()
{
	switch ((int)AppStateBot)
	{
	case 0: AppStateBot = 1; break;
	case 1: AppStateBot = 2; break;
	default: AppStateBot = 0; break;
	}

}

double appatureStartTime = 0;



void tglAppatureBegin()
{

	appatureStartTime = frc::GetClock();

	//if (LH_ShotAngBeg) LH_ShotAngBeg = 0; else LH_ShotAngBeg = 3;

}

void tglAppatureService()
{

	//if (frc::GetClock() > appatureStartTime + 1.0) LH_ShotAngBeg = 3;

}

void loadFrizCmd()
{

	//LH_ShotAngBeg = 2;
}

void WheelOnOffCmd()
{

	if (wantedShooterRPM == 0)
	{

		wantedShooterRPM = 4200;

	}
	else
		wantedShooterRPM = 0;


}



double wheelOnOffStartTime = 0;

void WheelOnOffBegin()
{

	wheelOnOffStartTime = frc::GetClock();

	if (wantedShooterRPM == 0)
	{

		wantedShooterRPM = 99; //4200;

	}
	else
		wantedShooterRPM = 0;

}

void WheelOnOffService()
{

	if (frc::GetClock() > wheelOnOffStartTime + 1.0) wantedShooterRPM = 4200;

}












void tglCamera()
{


	//iArm=111;sendBack(320,iArm);

}

void UnGrabCmd(uc i);

void GrabCmd(uc cmd)
{



}

void UnGrabCmd(uc i)
{

}


void autoLoadBegin()
{
	//armFlag=5; //send to safe location (very high)

	cmdOpenButterfly();

	armFlag = 0;
	armRate = 1;
	armState = 1; //send arm up at max rate no ramps

//	stopTimer(aArm);

	GrabCmd(2);



}


void autoLoadEnd()
{
	armFlag = 4;//send to Floor

	tIndex = 1;

	//tableSize=2;

	stopTimer(19);
}


int liftIsUp = 0;

void stopLift(uc i)
{
	//tableSize=0;
}


void liftUpCmd()
{

	//liftIsUp=1;

	//tableSize=1;

	//startTimer(19,liftTime*100,0,stopLift);

}

void liftDownCmd()
{

	//liftIsUp=0;

//	tableSize=0;

	//startTimer(19,liftTime*100,0,stopLift);

}

void liftTglCmd()
{

	if (liftIsUp) liftDownCmd(); else liftUpCmd();

}

void tglGyroCmd()
{
	if (holdStraight) holdStraight = 0; else holdStraight = 1;

	gyroShiftState = 0;
}

double beginTglTime = -1;
int    resetGyroFlag = 0;

void tglGyroBeg()
{
	tglGyroCmd();

	beginTglTime = frc::GetClock();

	resetGyroFlag = 0;

}

void tglGyroServie()
{

	double newTime = frc::GetClock();

	if (newTime > beginTglTime + 1.0 && !resetGyroFlag)
	{
		ZeroHood();	resetGyroFlag = 1; holdStraight = 0;
	}

	if (newTime > beginTglTime + 2.0 && resetGyroFlag)
	{
		holdStraight = 1;
	}


}

void tlgGyroEnd()
{



}





void CloseTheButterfly(uc i);


void cmdOpenButterfly()
{
	//  toroInManual=1;

	//  startTimer(19,200,0,CloseTheButterfly);

	 // startTimer(18,200,0,CloseTheButterfly);

}

void cmdCloseButterfly()
{
	toroInManual = 0;
}

void CloseTheButterfly(uc i)
{
	cmdCloseButterfly();
}

int cameraRangeFlag = 0;

void cameraRangeBeginCmd()
{

	cameraRangeFlag = 0;

}
void cameraRangeServiceCmd()
{
	int c1 = crossHairOffset / 4;
	int c2 = crossHair2Y / 4;

	if (c1 > c2) { int tmp = c1; c1 = c2; c2 = tmp; }

	int rangeMinTop = c1;
	int rangeMinBot = c2;


	if (cameraRangeFlag == 0)
	{
		if (minDisplayLine < rangeMinTop) minDisplayLine += 0.2;
		if (maxDisplayLine > rangeMinBot) maxDisplayLine -= 0.2;

		if (minDisplayLine >= rangeMinTop && maxDisplayLine <= maxDisplayLine)
		{
			cameraRangeFlag = 1;
		}

	}


	if (cameraRangeFlag == 1)
	{
		if (minDisplayLine > 0) minDisplayLine -= 0.2;
		if (maxDisplayLine < 120) maxDisplayLine += 0.2;

		if (minDisplayLine <= 0 && maxDisplayLine >= 120)
		{
			cameraRangeFlag = 0;
		}

	}

}
void cameraRangeEndCmd()
{

}

void ReverseWheel()
{

	wantedShooterRPM = -99;
}

void ForwardWheel()
{

	wantedShooterRPM = 99;

}


void WireUpBeginCmd()
{
	tprintf("Shoot The Ball\n");

	shooterState = 100;
}

void WireDnBeginCmd()
{
	//tprintf("Reset Shooter\n");

	//shooterState=2;

}

float beginTimeBase = 0;
float beginTimeEnd = 0;
float midTimeEnd = 0;
float topTimeEnd = 0;
float downTimeEnd = 0;
float endTimeEnd = 0;


//void setupShootTimes()
//{
//	tprintf("setupShootTimes\n");
//
//	float baseTime=GetClock();
//
//	beginTimeBase=baseTime;
//	beginTimeEnd =beginTimeBase+sShotAngle2;
//	midTimeEnd   =beginTimeEnd +shooterMidDuration;
//	topTimeEnd   =midTimeEnd   +shooterTopDuration;
//	downTimeEnd  =topTimeEnd   +shooterDownDuration;
//	endTimeEnd   =downTimeEnd  +shooterEndDuration;
//}
//
////ballLoadLoc
////ballLoadRate
////fShotAngle1
//
//float shotDelta=1.0;
//
//int getShooterLevel()
//{
//	float angle=loc[aArm];
//
//	//tprintf("angle %f\n",angle);
//
//	int level=0;
//
//	shotDelta=0.0;
//
//	if (fShotAngle1 && angle>fShotAngle1)
//		{
//
//		level=1;
//
//		if (ballLoadRate!=fShotAngle1)
//
//		     shotDelta=(angle-fShotAngle1)/(ballLoadRate-fShotAngle1);
//
//		     if (shotDelta>1.0) shotDelta=1.0;
//
//		}
//
//	if (ballLoadRate && angle>ballLoadRate) level=2;
//
//	if (ballLoadLoc && angle>ballLoadLoc) level=3;
//
//}
//
//float getShootRate()
//{
//	int curLevel=getShooterLevel();
//
//	//tprintf("curLevel %d\n",curLevel);
//
//	float curTime=GetClock();
//
//	if (curTime<beginTimeBase) { shooterState=0;  return 0.0; }
//
//	if (shooterState==10 && curLevel==2) //have reached mid Zone
//	{
//		    beginTimeEnd =curTime      -1;
//			midTimeEnd  =curTime      +shooterMidDuration;
//	        topTimeEnd   =midTimeEnd   +shooterTopDuration;
//	        downTimeEnd  =topTimeEnd   +shooterDownDuration;
//	        endTimeEnd   =downTimeEnd  +shooterEndDuration;
//
//	        shooterState=11;
//
//	        tprintf("sSTATE %2.0f curTime %4.2f bTEnd %4.2f\n",shooterState,curTime,beginTimeEnd);
//	}
//
//	if (shooterState==11 && curLevel==3) //have reached top Zone
//	{
//
//		    beginTimeEnd =curTime -1;
//		    midTimeEnd   =curTime -1;
//
//	        topTimeEnd   =curTime      +shooterTopDuration;
//	        downTimeEnd  =topTimeEnd   +shooterDownDuration;
//	        endTimeEnd   =downTimeEnd  +shooterEndDuration;
//
//	        shooterState=12;
//
//	        tprintf("SSTATE %2.0f curTime %4.2f bTEnd %4.2f\n",shooterState,curTime,beginTimeEnd);
//	}
//
//	//tprintf("SSTATE %2.0f curTime %4.2f bTEnd %4.2f\n",shooterState,curTime,beginTimeEnd);
//
//	if (curTime<beginTimeEnd )
//	{
//		shooterState=10;
//
//		return sShotAngle2+shotDelta*(sShotAngle1-sShotAngle2);
//	}
//	if (curTime<midTimeEnd   ) { shooterState=11; return sShotAngle1; }
//	if (curTime<topTimeEnd   ) { shooterState=12; return -fShotRate2; }
//	if (curTime<downTimeEnd  ) { shoZoterState=13; return -shooterDownRate; }
//	if (curTime<endTimeEnd   ) { shooterState=14; return -shooterEndRate; }
//
//	tprintf("Shot IS Finished\n");
//
//	shooterState=0;
//
//	return 0.0;
//
//}
//
//
//float getShootRatexxx()
//{
//	float curTime=GetClock();
//
//	if (curTime<beginTimeBase) { shooterState=0;  return 0.0; }
//
//	if (curTime<beginTimeEnd ) { shooterState=10; return sShotAngle2; }
//	if (curTime<midTimeEnd   ) { shooterState=11; return sShotAngle1; }
//	if (curTime<topTimeEnd   ) { shooterState=12; return fShotRate2; }
//	if (curTime<downTimeEnd  ) { shooterState=13; return -shooterDownRate; }
//	if (curTime<endTimeEnd   ) { shooterState=14; return -shooterEndRate; }
//
//	tprintf("ShotFinished\n");
//
//	shooterState=0;
//
//	return 0.0;
//
//}
void forceShot();

void SendTorrowsUp(void(*serviceAddress)());

void ShootBall(float ShotAngBeg, float ShotAngEnd, float ShotSpeed);

void ShotACmd2()
{
	tprintf("ShotACmd2\n");

	ShootBall(A_ShotAngBeg, A_ShotAngEnd, A_ShotSpeed);

	if (toroIsDown) forceShot();
}

void testTable();


void  CatcherOpenCmd();

void ShotACmd()
{
	if (A_ShotAngBeg < 10) { sonarShot(A_ShotAngBeg); return; }


	testTable();

	//if (!catcherIsOpen) CatcherOpenCmd();

	if (fabs(ballLoadLoc - loc[5]) > 10) { ShotACmd2(); return; }

	tprintf("ShotACmd\n");

	ShootBall(A_ShotAngBeg, A_ShotAngEnd, A_ShotSpeed);

	if (toroIsDown) forceShot();


	//SendTorrowsUp(ShotACmd2);
}

void ShotBCmd2()
{

	ShootBall(B_ShotAngBeg, B_ShotAngEnd, B_ShotSpeed);

	if (toroIsDown) forceShot();
}

void ShotBCmd()
{
	if (B_ShotAngBeg < 10) { sonarShot(B_ShotAngBeg); return; }


	//if (!catcherIsOpen) CatcherOpenCmd();

	tprintf("ShotBCmd\n");

	testTable();

	if (fabs(ballLoadLoc - loc[5]) > 10) { ShotBCmd2(); return; }
	ShootBall(B_ShotAngBeg, B_ShotAngEnd, B_ShotSpeed);

	if (toroIsDown) forceShot();

	//SendTorrowsUp(ShotBCmd2);
}

void ShotXCmd2()
{



	ShootBall(X_ShotAngBeg, X_ShotAngEnd, X_ShotSpeed);

	if (toroIsDown) forceShot();
}

void ShotXCmd()
{
	if (X_ShotAngBeg < 10) { sonarShot(X_ShotAngBeg); return; }

	testTable();

	//if (!catcherIsOpen) CatcherOpenCmd();

	tprintf("ShotXCmd\n");

	if (fabs(ballLoadLoc - loc[5]) > 10) { ShotXCmd2(); return; }
	ShootBall(X_ShotAngBeg, X_ShotAngEnd, X_ShotSpeed);

	if (toroIsDown) forceShot();

	//SendTorrowsUp(ShotXCmd2);
}

void ShotYCmd2()
{


	ShootBall(Y_ShotAngBeg, Y_ShotAngEnd, Y_ShotSpeed);

	if (toroIsDown) forceShot();
}


void ShotYCmd()
{
	if (Y_ShotAngBeg < 10) { sonarShot(Y_ShotAngBeg); return; }


	//if (!catcherIsOpen) CatcherOpenCmd();

	tprintf("ShotYCmd\n");
	testTable();
	if (fabs(ballLoadLoc - loc[5]) > 10) { ShotYCmd2(); return; }
	ShootBall(Y_ShotAngBeg, Y_ShotAngEnd, Y_ShotSpeed);

	if (toroIsDown) forceShot();

	// SendTorrowsUp(ShotYCmd);

}


void ShotRSCmd2()
{
	ShootBall(RS_ShotAngBeg, RS_ShotAngEnd, RS_ShotSpeed);

	if (toroIsDown) forceShot();
}


void ShotRSCmd()
{
	if (RS_ShotAngBeg < 10) { sonarShot(RS_ShotAngBeg); return; }

	//if (!catcherIsOpen) CatcherOpenCmd();
	tprintf("ShotRSCmd\n");
	testTable();
	if (fabs(ballLoadLoc - loc[5]) > 10) { ShotRSCmd2(); return; }
	ShootBall(RS_ShotAngBeg, RS_ShotAngEnd, RS_ShotSpeed);

	if (toroIsDown) forceShot();

}

void ShotLHCmd2()
{
	//ShootBall(LH_ShotAngBeg, LH_ShotAngEnd, LH_ShotSpeed);

	if (toroIsDown) forceShot();
}

void ShotLHCmd()
{
	//if (LH_ShotAngBeg < 10) { sonarShot(LH_ShotAngBeg); return; }


	//if (!catcherIsOpen) CatcherOpenCmd();

	tprintf("ShotHLCmd\n");
	testTable();
	if (fabs(ballLoadLoc - loc[5]) > 10) { ShotLHCmd2(); return; }
	//ShootBall(LH_ShotAngBeg, LH_ShotAngEnd, LH_ShotSpeed);

	if (toroIsDown) forceShot();

}

void ShotRHCmd2()
{
	ShootBall(RH_ShotAngBeg, RH_ShotAngEnd, RH_ShotSpeed);

	if (toroIsDown) forceShot();
}


void ShotRHCmd()
{
	if (RH_ShotAngBeg < 10) { sonarShot(RH_ShotAngBeg); return; }

	//if (!catcherIsOpen) CatcherOpenCmd();

	tprintf("ShotHRCmd\n");
	testTable();
	if (fabs(ballLoadLoc - loc[5]) > 10) { ShotRHCmd2(); return; }
	ShootBall(RH_ShotAngBeg, RH_ShotAngEnd, RH_ShotSpeed);

	if (toroIsDown) forceShot();
}

void initKeyState();

void Turn180();
void Spin180();

void ReverseDriveToggle();

void ReverseDriveToggleNoHold();


extern int reverseDrive;

void ReverseDriveCmd()
{
	reverseDrive = 1;
}

void ForwardDriveCmd()
{
	reverseDrive = 0;

}

extern float armMoveFlag;
extern float wantedArmAngle;

void toggleArmManualCmd()
{

	if (armManualFlag)
	{

		tprintf("Arm Manual Mode OFF\n");

		armManualFlag = 0;

	}
	else
	{
		tprintf("Arm Manual Mode ON\n");

		armManualFlag = 1;

		shooterState = 0;

		wantedArmAngle = 0;

		armMoveFlag = 0;
	}


}

void SendTorrowsUp(void(*serviceAddress)());
void SendTorrowsDown(void(*serviceAddress)());



void ToroUpCmdFinished()
{
	tprintf("ToroUpCmdFinished\n");

}

void ToroUpCmd()
{
	//SendTorrowsUp(ToroUpCmdFinished);

}


void ToroDownCmdFinished()
{
	tprintf("ToroDownCmdFinished\n");

}

void ToroDownCmd()
{
	//printf("ToroDownCmd\n");

	SendTorrowsDown(ToroDownCmdFinished);
}





void SendBumperDown(void(*serviceAddress)());
void SendBumperUp(void(*serviceAddress)());
void OpenCatcher(void(*serviceAddress)());
void CloseCatcher(void(*serviceAddress)());

void BumperDownCmdFinished()
{
	tprintf("BumperDownCmdFinished\n");
}
/*
void BumperDownCmd()
{
	SendBumperDown(BumperDownCmdFinished);
}

void BumperUpCmdFinished()
{
	tprintf("BumperUpCmdFinished\n");
}

void BumperUpCmd()
{
	SendBumperUp(BumperUpCmdFinished);

}

void OpenCatcherCmdFinished()
{
	tprintf("OpenCatcherCmdFinished\n");
}

void CatcherOpenCmd()
{
	OpenCatcher(OpenCatcherCmdFinished);
}

void CloseCatcherCmdFinished()
{
	tprintf("CloseCatcherCmdFinished\n");
}

void CatcherCloseCmd()
{
	CloseCatcher(CloseCatcherCmdFinished);

}
*/
extern float toroIsDown;

extern float toroIsGoingUp;
extern float toroIsGoingDown;


int torUp = 0;
int torDn = 0;

double toroUpMinTime = 0;
double toroDnMinTime = 0;

double toroToggleDownBeginTime = 0;

void toggleToroUpDown()
{
	//if (toroIsDown  || toroIsGoingDown ) {toroUpMinTime=GetClock()+toroUpTime;torUp=1; torDn=0;ToroUpCmd();}
	//else
	//{toroDnMinTime=GetClock()+toroDownTime;torUp=0; torDn=1;ToroDownCmd();}

	toroToggleDownBeginTime = frc::GetClock();

	if (toroIsDown) ToroUpCmd();
	else
		ToroDownCmd();
}
/*
void toggleToroUpDownService()
{
	double curTime = frc::GetClock();
	double dTime = curTime - toroToggleDownBeginTime;

	if (dTime > 1) { torUp = 0; torDn = 1; }

	if (torUp) SetToroUpSpeed(toroUpRate);
	if (torDn) SetToroDownSpeed(toroDownRate);

	blockToroAuto = 1;

}

void toggleToroUpDownFinish()
{

	if (torUp) if (frc::GetClock() > toroUpMinTime) { SetToroUpSpeed(0); toroIsUp = 1; toroIsDown = 0; }
	if (torDn) if (frc::GetClock() > toroDnMinTime) { SetToroDownSpeed(0); toroIsUp = 0; toroIsDown = 1; }

	blockToroAuto = 0;

}

*/

extern int	 bumperIsGoingDown;
extern int	 bumperIsGoingUp;

/*
void ToggleBumperUpCmd()
{
	if (bumperIsDown || bumperIsGoingDown) BumperUpCmd();
	else
		BumperDownCmd();

}


extern int catcherIsGoingOpen;
extern int catcherIsGoingClosed;


void ToggleCatcherOpenCloseCmd()
{
	if (catcherIsOpen || catcherIsGoingOpen) CatcherCloseCmd();
	else
		CatcherOpenCmd();

}
*/
void ButtonTest()
{
	printf("ButtonTest\n");

}

void TestA()
{

	printf("testA \n");

}

extern float  shooterLastSpeed;
extern double shooterLastSpeedTime;

int ShooterAtSpeed(float speed, float &timeLeft)
{
	//printf("sLastSpeed %f Speed %f\n",shooterLastSpeed,speed);

	timeLeft = 2.0;

	//printf("lSpeed %f speed %f lTime %f\n",shooterLastSpeed,speed,shooterLastSpeedTime);

	if (shooterLastSpeed != speed) return 0;
	if (shooterLastSpeedTime == 0) return 0;

	double curTime = frc::GetClock();
	double deltaTime = curTime - shooterLastSpeedTime;

	timeLeft = 2.0 - deltaTime;

	//printf("Cur %f last %f timeLeft %f\n",(float)curTime,(float)shooterLastSpeedTime,(float)timeLeft);

	//printf("delta Time %f Left %f\n",(float) deltaTime,timeLeft);

	if (deltaTime < 2.0) return 0;

	timeLeft = 0;

	return 1;
}



void SetSolenoid(int num, int state);

void SetWheelLevel(int front, int right, int raise)
{

	SetSolenoid(front * 2 + right, raise);

}



void ToolBarUp()
{
	printf("ToolBarUp\n");

	//Set_ToolBar(1);
	ToolBarAuto = 0;
}

void ToolBarDown()
{
	printf("ToolBarDown\n");

	//Set_ToolBar(-1);
	ToolBarAuto = 0;
}

void ToolBarStop()
{
	printf("ToolBarStop\n");

	//Set_ToolBar(0.0);
	ToolBarAuto = 0;
}


//void Set_FrontDriveRt       (float speed)  { if (blockCanIO != 1) { FrontDriveRt_Can      ->Set(speed); } pwm[ FrontDriveRt_Num     ] = speed; };
//void Set_RearDriveRt        (float speed)  { if (blockCanIO != 1) { RearDriveRt_Can       ->Set(speed); } pwm[ RearDriveRt_Num		] = speed; };
//void Set_WinchMotor1		(float speed)  { if (blockCanIO != 1) { WinchMotor1_Can		  ->Set(speed); } pwm[ WinchMotor1_Num      ] = speed; };
//void Set_ToolBar			(float speed)  { if (blockCanIO != 1) { ToolBar_Can			  ->Set(speed); } pwm[ ToolBar_Num 			] = speed; };
//void Set_HeadLights			(float speed)  { if (blockCanIO != 1) { HeadLights_Can		  ->Set(speed); } pwm[ HeadLights_Num 		] = speed; };
//void Set_ShooterElevation	(float speed)  { if (blockCanIO != 1) { ShooterElevation_Can  ->Set(speed); } pwm[ ShooterElevation_Num	] = speed; };
//void Set_ToroRight			(float speed)  { if (blockCanIO != 1) { ToroRight_Can		  ->Set(speed); } pwm[ ToroRight_Num 		] = speed; };
//void Set_BallFeed			(float speed)  { if (blockCanIO != 1) { BallFeed_Can		  ->Set(speed); } pwm[ BallFeed_Num 		] = speed; };
//void Set_SpareTalon1		(float speed)  { if (blockCanIO != 1) { SpareTalon1_Can		  ->Set(speed); } pwm[ SpareTalon1_Num 		] = speed; };
//void Set_ToroLeft			(float speed)  { if (blockCanIO != 1) { ToroLeft_Can		  ->Set(-speed); } pwm[ ToroLeft_Num 		] = speed; };
//void Set_ShooterBallShoe	(float speed)  { if (blockCanIO != 1) { ShooterBallShoe_Can	  ->Set(speed); } pwm[ ShooterBallShoe_Num 	] = speed; };
//void Set_ShareTalon2		(float speed)  { if (blockCanIO != 1) { ShareTalon2_Can		  ->Set(speed); } pwm[ ShareTalon2_Num 		] = speed; };
//void Set_ShooterWheel		(float speed)  { if (blockCanIO != 1) { ShooterWheel_Can	  ->Set(speed); } pwm[ ShooterWheel_Num 	] = speed; };
//void Set_WinchMotor2		(float speed)  { if (blockCanIO != 1) { WinchMotor2_Can		  ->Set(speed); } pwm[ WinchMotor2_Num 		] = speed; };
//void Set_RearDriveLt		(float speed)  { if (blockCanIO != 1) { RearDriveLt_Can		  ->Set(speed); } pwm[ RearDriveLt_Num 		] = speed; };
//void Set_FrontDriveLt		(float speed)  { if (blockCanIO != 1) { FrontDriveLt_Can	  ->Set(speed); } pwm[ FrontDriveLt_Num 	] = speed; };

/*
void StopTheBall();

void StopTheBall(unsigned char i)
{
	StopTheBall();
}

void FeedTheBall(unsigned char)
{
	printf("FeedTheBall\n");

	Set_BallFeed(1.0);
	startTimer(1, 100, 0, StopTheBall);
}

void QuickBallFeed()
{
	printf("QuickTheBall\n");

	Set_BallFeed(1.0);
	startTimer(1, 100, 0, StopTheBall);
}

void ShootTheBall()
{
	printf("ShootTheBall\n");

	stopTimer(1);

	float timeLeft = 0;

	if (ShooterAtSpeed(1.0, timeLeft)) { FeedTheBall(1); return; }

	Set_ShooterWheel(1.0);

	printf("Shooter Speed Wait %f\n", timeLeft);

	startTimer(1, timeLeft * 100, 0, FeedTheBall);
}


void ShootTheBallLow()
{
	printf("ShootTheBallLow\n");

	stopTimer(1);

	float timeLeft = 0;

	if (ShooterAtSpeed(LowGoalSpeed, timeLeft)) { FeedTheBall(1); return; }

	Set_ShooterWheel(LowGoalSpeed);

	printf("Shooter Speed Wait %f\n", timeLeft);

	startTimer(1, timeLeft * 100, 0, FeedTheBall);
}

void CollectTheBall()
{
	printf("CollectTheBall\n");

	Set_BallFeed(-0.5);

	Set_ShooterWheel(-0.6);
}

void StopTheBall()
{
	printf("StopTheBall\n \n");

	Set_BallFeed(0.0);

	Set_ShooterWheel(0);
}

void ShoeDown()
{
	ShoeAuto = 0;
	Set_ShooterBallShoe(0.5);
}

void StopShoe()
{
	printf("StopSHoe\n");

	StopShoeAuto();
}

void ShoeUp()
{
	ShoeAuto = 0;
	Set_ShooterBallShoe(-0.5);
}




void MoveToolBar(float dest);

void MoveToolBarLowCmd()
{
	MoveToolBar(ToolBarLow);
}

void MoveToolBarMidCmd()
{
	MoveToolBar(ToolBarMid);
}

void MoveToolBarHighCmd()
{
	MoveToolBar(ToolBarHigh);
}


void MoveElev(float dest);

void MoveElevLowCmd()
{
	MoveElev(ElevLow);
}

void MoveElevMidCmd()
{
	MoveElev(ElevMid);

}

void MoveElevHighCmd()
{

	MoveElev(ElevHigh);

}

void MoveShoe(float dest);

void MoveShoeLowCmd()
{
	MoveShoe(ShoeLow);
}

void MoveShowMidCmd()
{
	MoveShoe(ShoeMid);

}

void MoveShoeHighCmd()
{

	MoveShoe(ShoeHigh);

}

void CloseShoe()
{
	ShoeDest = ShoeLow;
	ShoeAuto = 1;

}
void OpenShoe()
{


}

void MoveShoeToLow()
{
	ShoeDest = ShoeLow;
	ShoeAuto = 1;
}
void MoveShoeToMid()
{
	ShoeDest = ShoeMid;
	ShoeAuto = 1;
}
void MoveShoeToHigh()
{
	ShoeDest = ShoeHigh;
	ShoeAuto = 1;
}

void StopElevAuto()
{
	ElevAuto = 0;
	ElevVolts = 0;
	Set_ShooterElevation(0);
}

void StopToolBarAuto()
{
	ToolBarAuto = 0;
	ToolBarVolts = 0;
	Set_ToolBar(0);
}

void StopShoeAuto()
{
	ShoeAuto = 0;
	ShoeVolts = 0;
	Set_ShooterBallShoe(0);
}

void StopAllAuto()
{
	StopElevAuto();
	StopToolBarAuto();
	StopShoeAuto();
}



void MoveElevToLow()
{

	ElevDest = ElevLow;
	ElevAuto = 1;


}
void MoveElevToMid()
{

	ElevDest = ElevMid;
	ElevAuto = 1;


}

void MoveElevToMid2()
{

	ElevDest = ElevMid2;
	ElevAuto = 1;


}
void MoveElevToHigh()
{

	ElevDest = ElevHigh;
	ElevAuto = 1;


}

void MoveToolBarToLow()
{
	printf("MoveToolBarToLow\n");

	ToolBarDest = ToolBarLow;
	ToolBarAuto = 1;
}

void MoveToolBarToMid()
{
	printf("MoveToolBarToMid\n");

	ToolBarDest = ToolBarMid;
	ToolBarAuto = 1;
}

void MoveToolBarToHigh()
{
	printf("MoveToolBarToHigh\n");

	ToolBarDest = ToolBarHigh;
	ToolBarAuto = 1;
}

int ForceToroCollect = 0;

void ToroCollectOn()
{

	ForceToroCollect = 1;

}

void ToroCollectOnFast()
{

	ForceToroCollect = 2;

}

void ReadyToLoad()
{

	stopTimer(1);

	MoveShoeToHigh();
	MoveElevToLow();
	MoveToolBarToMid();
	CollectTheBall();
	ToroCollectOn();


}

void LowerElevator()
{
	MoveShoeToHigh();
	MoveElevToLow();
}

void LoadIsFinished(unsigned char i)
{
	StopTheBall();

	StopAllAuto();

	ForceToroCollect = 0;


	// MoveToolBarToMid();
}

void ResetToolBar()
{
	MoveToolBarToHigh();
}

void ToroCollectOnFast();

void LoadTheBall()
{
	printf("LoadTheBall\n");

	MoveToolBarToHigh();



	ElevAuto = 0;

	ToroCollectOnFast();

	startTimer(1, 250, 0, LoadIsFinished);
}

void ReadyToShoot()
{
	printf("ReadyToShoot\n");

	MoveToolBarToMid();
	MoveElevToHigh();
	MoveShoeToLow();

	Set_ShooterWheel(1.0);
}

void Duck()
{
	printf("Duck\n");

	stopTimer(1);

	MoveToolBarToLow();
	MoveElevToLow();
	MoveShoeToMid();
}

void DuckStop()
{
	printf("Duck Stop\n \n");

	StopAllAuto();
}

void LowGoalSetup()
{
	printf("LowGoalSetup\n");

	stopTimer(1);

	MoveToolBarToMid();
	MoveElevToLow();
	MoveShoeToLow();

	if (LowGoalSpeed < 0.3) LowGoalSpeed = 1;

	Set_ShooterWheel(LowGoalSpeed);
}

void LowGoalShoot()
{
	printf("LowGoalShot\n");

	StopAllAuto();

	stopTimer(1);

	if (ToolBarLoc >= 45)
	{
		ShootTheBallLow();
	}
	else
	{
		printf("ToolBar < 45 (Bar To High) %f\n", ToolBarLoc);

	}
}
void HighGoalSetup()
{
	printf("HighGoalSetup\n");

	stopTimer(1);

	MoveElevToHigh();
	MoveShoeToLow();
	Set_ShooterWheel(1.0);
}

void HighGoalShoot()
{
	printf("HighGoalShot\n");

	StopAllAuto();

	ShootTheBall();
}
*/
extern Servo *hangServo;


void HangServoToZero()
{
	//hangServo->SetAngle(0);
}

void HangServoToNinty()
{
	//hangServo->SetAngle(180);
}

float ffabs(float val);

float CustomDriveFlag = 1;

//---------------------------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------

/*
void StartGoalTracking()
{
	printf("StartGoalTracking\n");

	LookForGoal = 1;

	//RaiseWheels();

	MoveElevToHigh();

	MoveShoeToLow();

	Set_ShooterWheel(1.0);
}

void StartGoalTracking3()
{
	printf("StartGoalTracking3\n");

	LookForGoal = 1;

	//RaiseWheels();

	MoveElevToMid();

	MoveShoeToLow();

	Set_ShooterWheel(1.0);
}


void StartGoalTracking4()
{
	printf("StartGoalTracking4\n");

	LookForGoal = 1;

	RaiseWheels();

	MoveElevToMid2();

	MoveShoeToLow();

	Set_ShooterWheel(1.0);
}

void TrackGoal2()
{
	if (GoalFound)
	{

		gTimeOut = 1000;

		int goalErrorX = GoalX - CrossX2 * 2;

		float volts = -goalErrorX / 30.0;

		if (volts < 0) volts = -(volts*volts); else volts = volts*volts;

		if (volts < -0.5)volts = -0.5;
		if (volts > 0.5) volts = 0.5;

		autoDriveR = -volts*TrackingGain;

		if (ffabs(autoDriveY) > 0.05) autoDriveR /= 3;

		int goalErrorY = GoalY - CrossY2 / 2;

		volts = -goalErrorY / 20.0;

		if (volts < 0) volts = -(volts*volts); else volts = volts*volts;

		if (volts < -0.5) volts = -0.5;
		if (volts > 0.5) volts = 0.5;

		autoDriveY = -volts*TrackingGainY;

		double dTime = frc::GetClock() - beginAtonTime;

		if (inAton && dTime > sBlockTime)
		{
			autoDriveX = 0;

			xDriftCount = 0;

			return;
		}


		autoDriveX = sGain*goalAngle*(1.0 - fabs(volts) * 2);

		xDriftCount--;

	}
	else
	{
		autoDriveR = 0;
		autoDriveX = 0;

		autoDriveY += 0.01; if (autoDriveY > CreapSpeed) autoDriveY = CreapSpeed;
		if (xDriftCount > 0) autoDriveX = xDrift;

	}
}

void TrackGoal1()
{
	if (GoalFound)
	{

		gTimeOut = 1000;

		int goalErrorX = GoalX - CrossX1 * 2;

		float volts = -goalErrorX / 30.0;

		if (volts < 0) volts = -(volts*volts); else volts = volts*volts;

		if (volts < -0.5)volts = -0.5;
		if (volts > 0.5) volts = 0.5;

		autoDriveR = -volts*TrackingGain;

		if (ffabs(autoDriveY) > 0.05) autoDriveR /= 3;

		int goalErrorY = GoalY - CrossY1 / 2;

		volts = -goalErrorY / 20.0;

		if (volts < 0) volts = -(volts*volts); else volts = volts*volts;

		if (volts < -0.5) volts = -0.5;
		if (volts > 0.5) volts = 0.5;

		autoDriveY = -volts*TrackingGainY;

		double dTime = frc::GetClock() - beginAtonTime;

		if (inAton && dTime > sBlockTime)
		{
			autoDriveX = 0;

			xDriftCount = 0;

			return;
		}


		autoDriveX = sGain*goalAngle*(1.0 - fabs(volts) * 2);

		if (autoDriveX < -0.7) autoDriveX = -0.7;
		if (autoDriveX > 0.7) autoDriveX = 0.7;


		xDriftCount--;
	}
	else
	{
		autoDriveR = 0;
		autoDriveX = 0;

		autoDriveY += 0.01; if (autoDriveY > CreapSpeed) autoDriveY = CreapSpeed;
		if (xDriftCount > 0) autoDriveX = xDrift;

	}
}

void TrackGoal3()
{
	if (GoalFound)
	{

		gTimeOut = 1000;

		int goalErrorX = GoalX - CrossX3 * 2;

		float volts = -goalErrorX / 30.0;

		if (volts < 0) volts = -(volts*volts); else volts = volts*volts;

		if (volts < -0.5)volts = -0.5;
		if (volts > 0.5) volts = 0.5;

		autoDriveR = -volts*TrackingGain;

		if (ffabs(autoDriveY) > 0.05) autoDriveR /= 3;

		int goalErrorY = GoalY - CrossY3 / 2;

		volts = -goalErrorY / 20.0;

		if (volts < 0) volts = -(volts*volts); else volts = volts*volts;

		if (volts < -0.5) volts = -0.5;
		if (volts > 0.5) volts = 0.5;

		autoDriveY = -volts*TrackingGainY;

		double dTime = frc::GetClock() - beginAtonTime;

		if (inAton && dTime > sBlockTime)
		{
			autoDriveX = 0;

			xDriftCount = 0;

			return;		}


		autoDriveX = sGain*goalAngle*(1.0 - fabs(volts) * 2);

		if (autoDriveX < -0.7) autoDriveX = -0.7;
		if (autoDriveX > 0.7) autoDriveX = 0.7;

		xDriftCount--;
	}
	else
	{
		autoDriveR = 0;
		autoDriveX = 0;

		autoDriveY += 0.01; if (autoDriveY > CreapSpeed) autoDriveY = CreapSpeed;

		if (xDriftCount > 0) autoDriveX = xDrift;


	}
}


void TrackGoal4()
{
	if (GoalFound)
	{
		gTimeOut = 1000;

		int goalErrorX = GoalX - CrossX4 * 2;

		float volts = -goalErrorX / 30.0;

		if (volts < 0) volts = -(volts*volts); else volts = volts*volts;

		if (volts < -0.5)volts = -0.5;
		if (volts > 0.5) volts = 0.5;

		autoDriveR = -volts*TrackingGain;


		if (ffabs(autoDriveY) > 0.05) autoDriveR /= 3;

		int goalErrorY = GoalY - CrossY4 / 2;

		volts = -goalErrorY / 20.0;

		if (volts < 0) volts = -(volts*volts); else volts = volts*volts;

		if (volts < -0.5) volts = -0.5;
		if (volts > 0.5) volts = 0.5;

		autoDriveY = -volts*TrackingGainY;


		double dTime = frc::GetClock() - beginAtonTime;

		if (inAton && dTime > sBlockTime)
		{
			autoDriveX = 0;

			xDriftCount = 0;

			return;
		}


		autoDriveX = sGain*goalAngle*(1.0 - fabs(volts) * 2);

		if (autoDriveX < -0.7) autoDriveX = -0.7;
		if (autoDriveX > 0.7) autoDriveX = 0.7;


		xDriftCount--;
	}
	else
	{
		autoDriveR = 0;

		autoDriveY += 0.01; if (autoDriveY > CreapSpeed) autoDriveY = CreapSpeed;
		if (xDriftCount > 0) autoDriveX = xDrift;
	}
}
void StopGoalTracking()
{
	LookForGoal = 0;

	GoalFound = 0;

	GoalH = 0;
	GoalW = 0;
	GoalX = 160;
	GoalY = 120;

	autoDriveR = 0;
	autoDriveY = 0;
	autoDriveX = 0;
}

*/
//-----------------------------------------------------------------------------------------------------

extern int gTimeOut;
extern float gyroZero;

/*
void xboxUpMessage()
{
	HighGoalSetup();

};

void xboxDownMessagfe()
{
	LowGoalSetup();

};

int gyroRotateFlag = 0;

void xboxRightMessage()
{
	RaiseWheels();

	gTimeOut = -200;

	gyroZero += 180.0;

	gyroRotateFlag = 1;
};

void xboxLeftMessage()
{
	RaiseWheels();

	gTimeOut = -200;

	gyroZero -= 180.0;

	gyroRotateFlag = 1;
};


void FinalRotate(unsigned char i)
{
	autoDriveY = 0;        // stop backup

	gTimeOut = -300;

	gyroZero -= AtonFinalRotate;

	gyroRotateFlag = 1;   //start 180deg rotation for max 3 sec
}

void RotateToDefences(unsigned char i)
{
	StopTheBall();

	LowerWheels();

	autoDriveY = -0.5; //need to backaway from tower

	LowerElevator();

	startTimer(1, 50, 0, FinalRotate);

}

void ToggleToolBar()
{

	if (ToolBarLoc >= 45)MoveToolBarToHigh();
	else MoveToolBarToMid();


}

void startWheelCmd()
{

	Set_ShooterWheel(1.0);

}
*/

void SetupCustomDrive()
{
	//Shooter Setup
	//onTrig(1, xboxA, LowGoalSetup, 0, LowGoalShoot);     //Low Goal Shoot - ShooterA
	//onTrig(1, xboxB, ShootTheBall, 0, 0);   //Shoot (Manual) - ShooterB
   // onTrig(1, xboxX, PassTheBall, 0, StopTheBall);       //Pass The Ball - ShooterX

   // onTrig(1,xboxX, QuickBallFeed,0,0);


	//onTrig(1, xboxX, ShootTheBall, 0, 0);

	////onTrig(1, xboxY, HighGoalSetup, 0, HighGoalShoot);   //High Goal Shoot - ShooterY


	//onTrig(1, xboxLS, Duck, 0, DuckStop);                //Duck - ShooterLB
	//onTrig(1, xboxRS, ReadyToLoad, 0, LoadTheBall);      //Load Configuration - ShooterRB

	//onTrig(1, xboxStart, StopTheBall, 0, 0);             //Stop The Ball - ShooterStart

   // onTrig(1, xboxLeftStickClick, HangServoToZero,0,0);
   // onTrig(1, xboxRightStickClick, HangServoToNinty,0,0);

	//onTrig(1, xboxRightStickClick, StartGoalTracking, TrackGoal2, StopGoalTracking);
	//onTrig(1, xboxLeftStickClick, StartGoalTracking3, TrackGoal3, StopGoalTracking);

	//onTrig(1, xboxBack, startWheelCmd, 0, 0);

	//onTrig(1, xboxUp, xboxUpMessage, 0, 0);
	//onTrig(1, xboxRight, xboxRightMessage, 0, 0);
	//onTrig(1, xboxDown, xboxDownMessage, 0, 0);
	//onTrig(1, xboxLeft, xboxLeftMessage, 0, 0);

	//Driver Setup
	//onTrig(2, xboxA, LowerWheels, 0, 0);                //Lower Wheels - DriverA
	//onTrig(2, xboxB, ReadyToShoot,0, 0);
	//onTrig(2, xboxY, RaiseWheels, 0, 0);                //Raise Wheels - DriverY

	 //onTrig(2, xboxX, QuickBallFeed, 0, 0);

	//onTrig(2, xboxX, ShootTheBall, 0, 0);

	//onTrig(2, xboxLS, Duck, 0, DuckStop);               //Duck - DriverLB
	//onTrig(2, xboxRS, ReadyToLoad, 0, LoadTheBall);     //Load Configuration - DriverRB

	//onTrig(2, xboxBack, ToolBarUp, 0, ToolBarStop);   //ToolBarDown (Manual) - DriverBack
	//onTrig(2, xboxStart, ToolBarDown, 0, ToolBarStop); //ToolBarUp (Manual) - DriverStart

   // onTrig(2, xboxLeftStickClick, ResetToolBar,0,0);
	//onTrig(2, xboxRightStickClick, StartGoalTracking, TrackGoal2, StopGoalTracking);
	//onTrig(2, xboxLeftStickClick, StartGoalTracking3, TrackGoal3, StopGoalTracking);

	//onTrig(2, xboxB, StartGoalTracking4, TrackGoal4, StopGoalTracking);

	///onTrig(2, xboxUp, MoveToolBarToHigh, 0, 0); //xboxUpMessage
	//onTrig(2, xboxRight, xboxRightMessage, 0, 0);
	//onTrig(2, xboxDown, MoveToolBarToMid, 0, 0);
	//onTrig(2, xboxLeft, xboxLeftMessage, 0, 0);
}

int ForceBallIntake;
int ForceHopperSpeed;
int ForceWinchSpeed;
int ForceTurretSpeed;

int ForceToolBarUp=0;

int ForceShoot=0;
void ManualShoot(){
	ShootingBalls();

}

void MoveAntiBackDriveToPOS(float pos);
float  Get_AntiBackDrivePOS();
void Set_Shooter(float rpm);
void customJoySticks()
{
	if (p_sw[2][13]){
			ManualShoot();
			ForceShoot=1;
		}
	else if(ForceShoot){
		StopShootingBalls();
		ForceShoot=0;
	}
	if (p_sw[2][14]){
		printf("ManualMoveShooterLeft\n");
		ForceTurretSpeed = 1;
		Set_Turret(0.5);
	}

	else if (p_sw[2][12]){
		printf("ManualMoveShooterRight\n");
		ForceTurretSpeed = 1;
		Set_Turret(-0.5);
		}

	else if(ForceTurretSpeed){
		Set_Turret(0);
		ForceTurretSpeed = 0;
		}



	if (p_sw[1][14]){
			printf("ManualMoveHopperLeft\n");
			Set_Hopper(0.7);
			ForceHopperSpeed = 1;
		}
	else if (p_sw[1][12]){
				printf("ManualMoveHopperRight\n");
				ForceHopperSpeed = 1;
				Set_Hopper(-0.7);
			}

		else{
			if(ForceHopperSpeed){
				Set_Hopper(0);
				ForceHopperSpeed = 0;
			}
		}
	float joystick1trigger = joyT[1];

	if(fabs(joystick1trigger) > 0.2){
		ForceBallIntake = 1;
		printf("ManualBallIntake: %f\n", joystick1trigger);
		Set_BallIntake(joystick1trigger);
	}
	else if(ForceBallIntake){
			Set_BallIntake(0);
			ForceBallIntake = 0;
		}

	float joystick2trigger = joyT[2];

	if(fabs(joystick2trigger) > 0.2){

		ForceWinchSpeed = 1;
		printf("ManualWinch: %f\n", joystick2trigger);
		Set_Winch(joystick2trigger);
		}
		else if(ForceWinchSpeed){
				Set_BallIntake(0);
				ForceWinchSpeed = 0;


			}

	//setAntiBackDrive(joystick2trigger*180);

}

void MoveHopperToPOS80(){
	MoveHopperToPOS(80);
	printf("Moveoppertopos80\n");
}
int oncce = 1;
int reverseHopper=0;

void StopFeedBalls()
{

	Set_Hopper(0);
}
void ReverseHopper(unsigned char i){
	printf("reverse Hopper\n");
	SetHopperToRPM(-hopperShootRPM+(50/(hopperShootRPM/(fabs(hopperShootRPM)))));
	reverseHopper=1;
	oncce=0;
}
float Get_HopperSetPoint();

int hopperStuckCount=0;
void resetRH(unsigned char i){
	reverseHopper=0;
	oncce=0;
	hopperStuckCount=0;
}
void ReverseHopper2(){
	Set_Hopper(-1);
	printf("reversehopper2\n");
}
void aimAndShoot(){
	selectCamera=2;
	SetShooterToRPM(ShooterShootRPM);
	GoalAim();
	float delta = BoilerGoalX - BoilerGoalX1;
	float ydelta = BoilerGoalY - BoilerGoalY1;
	if(abs(delta)<40 && abs(ydelta)<40){
		/*
		if(HopperAmps>8 && Get_HopperSetPoint()==hopperShootRPM && oncce>300 && hopperStuckCount>100){
			hopperStuckCount=0;
			oncce=0;
			startTimer(17, 75, ReverseHopper, resetRH);
		}
		else if(HopperAmps>8){
			hopperStuckCount++;
		}
		else{
			if(!reverseHopper){
				ShootingBalls();
				oncce++;
			}
		}
		*/
		if(HopperAmps>10 &&  Get_HopperSetPoint()==hopperShootRPM){
			//ReverseHopper2();
		}
		else{
			ShootingBalls();
		}



	}
	else{
		printf("not close enough\n");

		DontShootBalls();
	}
}
void resetAimAndShoot();
void switchDriveMode();

void StopHopper(){
	Set_Hopper(0);
}
void xy(){
	stopTimer(10);
}

void ToggleIntakeOn(){
	Set_BallIntake(-0.7);
}
void ToggleIntakeOff(){
	Set_BallIntake(0);
}
void EngageAntiBackDrive();
void DegageAntiBackDrive();
void StopAntiBackDrive();
void xx(){
	selectCamera=1;
}
void stopGearSpit(unsigned char i){
	Set_GearIntake(0);
	stopTimer(10);
}
void SpitGear(){
	Set_GearIntake(-1);
	stopTimer(10);

	startTimer(10,100,0,stopGearSpit);
}
void setupdriving2017(){

	//DRIVER CONTROLS
	//onTrig(1,xboxY,ManualClimbUp,0,ManualClimbStop);
	//onTrig(1,xboxA,ManualClimbDown,0,ManualClimbStop);

	onTrig(1,xboxY,EngageAntiBackDrive,0,StopAntiBackDrive);
	onTrig(1,xboxA,DegageAntiBackDrive,0,StopAntiBackDrive);

	onTrig(1,xboxRS,StartLoadGear,xx,StopLoadGear);
	onTrig(1,xboxLS,StartDropGear,xx,StopDropGear);
	onTrig(1,xboxStart,0,MoveToolBarUp,StopToolBar);
	onTrig(1,xboxBack,0,MoveToolBarDown,StopToolBar);

	//onTrig(1,xboxB,resetAimAndShoot,aimAndShoot,StopShootingBalls);

	onTrig(1,xboxRightStickClick ,stopPegAim, PegAim,stopPegAim);

	onTrig(1,xboxLeftStickClick,stopGoalAim,GoalAim,stopGoalAim);

	onTrig(1,xboxX, switchCam,0,0);

	onTrig(1,xboxB, switchDriveMode, 0,0);



	//SHOOTER CONTROLS
	onTrig(2,xboxY,ToggleIntakeOn,0,0);
	onTrig(2,xboxA,ToggleIntakeOff,0,0);

	onTrig(2,xboxX, switchCam,0,0);
	//onTrig(2,xboxX, ShootBalls, 0, StopHopper);

	onTrig(2,xboxRS,0,ManualShoot,StopShootingBalls);
	onTrig(2,xboxLS,StartLoadGear,xy,SpitGear);

	onTrig(2,xboxStart,0,MoveToolBarUp,StopToolBar);
	onTrig(2,xboxBack,0,MoveToolBarDown,StopToolBar);

	onTrig(2,xboxB,resetAimAndShoot,aimAndShoot,StopShootingBalls);
}
//---------------------------------------------------------------------------------------------------------------------------------
void setupDriving(void)
{
	printf("setupDriving\n");

	initKeyState();

	setupdriving2017();

	//onTrig(2, xboxY, printmessage, 0, 0);
	/*
		onTrig(2,xboxY,RaiseWheels,0,0);
		onTrig(2,xboxA,LowerWheels,0,0);

		onTrig(2,xboxRS,ToolBarDown,0,ToolBarStop);
		onTrig(2,xboxLS,ToolBarUp  ,0,ToolBarStop);

		//onTrig(1,xboxStart,MoveShoeToHigh,0,0);
		//onTrig(1,xboxBack ,MoveShoeToLow,  0,0);

		onTrig(1,xboxA,ShootTheBall,0,0);
		onTrig(1,xboxB,CollectTheBall,0,StopTheBall);

		onTrig(2,xboxStart,ShoeDown,0,StopShoe);
		onTrig(2,xboxBack ,ShoeUp,  0,StopShoe);

		onTrig(1,xboxStart,ShoeDown,0,StopShoe);
		onTrig(1,xboxBack ,ShoeUp,  0,StopShoe);


		onTrig(1,xboxRightStickClick ,MoveElevLowCmd ,  0,0);
		f(1,xboxX               ,MoveElevMidCmd ,  0,0);
		onTrig(1,xboxY               ,MoveElevHighCmd,  0,0);

		//onTrig(2,xboxX,ReadyToLoad,0,LoadTheBall);
		onTrig(2,xboxB,ReadyToShoot,0,0);

		onTrig(2,xboxRS, ReadyToLoad,0,LoadTheBall);
		onTrig(1,xboxLS, Duck,0,DuckStop);
		onTrig(1,xboxA, LowGoalSetup, 0, LowGoalShoot);

		onTrig(1,xboxY, HighGoalSetup, 0, HighGoalShoot);
	*/
	//  if (CustomDriveFlag)
	  //{

	//SetupCustomDrive();

	// }

	return;

	// driver controls

	// left stick drives robot

	//onTrig(1,xboxLS,0,0,startAtonSelected);

	//onTrig(1,xboxLeftStickClick,NextAton   ,0,0);


	// arm operator buttons

	//left  stick arm updown
	//right stick Azimuth

   //  onTrig(2,xboxRS,0,spitBallService,spitBallEnd);

   //  onTrig(2,xboxA,PickupCarryToggle,0,0);
   //  onTrig(2,xboxB,MoveToBotFull    ,0,0);
	// onTrig(2,xboxX,MoveToMidFull    ,0,0);
   //  onTrig(2,xboxY,MoveToTopFull    ,0,0);

	 //onTrig(2,xboxLS,StartSuck,0,StopRoller);
	// onTrig(2,xboxRS,StartSpit,0,StopRoller);

	// onTrig(2,xboxLS,SuperSpit,0,StartSuck);

	// onTrig(2,xboxRS,StartManualSpit,0,StopRoller);

	// onTrig(2,xboxStart,miniBotTrigger,0,0);
	// onTrig(2,xboxBack ,miniBotReset,0,0);

	// onTrig(2,xboxStart,delayedMiniBot,0,0); //miniBotReady  ,0,0);
	// onTrig(2,xboxBack ,miniBotToggle,0,0);

	 // onTrig(2,xboxStart,miniBotReset  ,0,0);
	 // onTrig(2,xboxBack ,miniBotTrigger,0,0);


	//onTrig(2,xboxStart,reduceAtonDelay  ,0,0);
	//onTrig(2,xboxBack ,increaseAtonDelay,0,0);

	// onTrig(2,xboxLeftStickClick ,NextAton,0,0);
	// onTrig(2,xboxRightStickClick,increaseAtonDelay,0,0);

	// onTrig(2,xboxRightStickClick,AutoHang,0,0);

	//------------ testing on stick 4 -------------

	// onTrig(4,xboxA,StopRollerDelayed,0,0);
	// onTrig(4,xboxB,StartSuck        ,0,0);

	// onTrig(4,xboxA,StartTracking,0,0);
	// onTrig(4,xboxB,StopTracking ,0,0);

	// onTrig(4,xboxA,toggleRightShift,0,0);
	// onTrig(4,xboxB,toggleLeftShift ,0,0);


	//----------- 2012 new stuff ----------

	// xboxA
	// xboxB
	// xboxX
	// xboxY
	// xboxLS
	// xboxRS
	// xboxBack
	// xboxStart
	// xboxLeftStickClick
	// xboxRightStickClick


	//onTrig(2,xboxB,moveRobotStart,moveRobotService,moveRobotStop);

	// joystick one (driving)
	// joystick two (manual turret moving - no vision or small adjustments)

	// collector spit
	// tower spit
	// tower collect
	// tower shoot one   (when ready)
	// tower rapid shoot (ready or not)
	// auto aim off
	// next aton
	// start aton
	// shoot from key (fixed settings)

	//onTrig(2,xboxRS,0,ArmDown,ArmStop);
	//onTrig(2,xboxLS,0,ArmUp,ArmStop  );



	/*

	onTrig(2,xboxRS,FireTheBallCmd,FireTheBallCmdService,FireTheBallCmdEnd  );
	onTrig(2,xboxLS,spitBallsBeg,spitBallsService,CollectBallsTgl );


	onTrig(2,xboxLeftStickClick,0,0,tglShiftModeCmd );

	onTrig(2,xboxA,manualSetA,manualSetService,manualSetEnd);
	onTrig(2,xboxB,manualSetB,manualSetService,manualSetEnd);
	onTrig(2,xboxX,manualSetX,manualSetService,manualSetEnd);
	onTrig(2,xboxY,manualSetY,manualSetService,manualSetEnd);

	onTrig(2,xboxBack,0,0 ,driverSelectCmd);

	onTrig(2,xboxRightStickClick  ,toggleAutoTracking,0,0);

	onTrig(2,xboxStart  ,togg
	leTurretTracking,0,0);

	*/

	//onTrig(2,xboxRightStickClick,0,0,liftTglCmd );


//	onTrig(2,xboxStart,feederOnCmd,0,toroBlockAfterShotTimeCmd);
//	onTrig(2,xboxA    ,feederSpeedOnCmd,feederSpeedServiceCmd,feederSpeedOffCmd);
	//onTrig(2,xboxStart,kickVideo,0,0);
	//onTrig(2,xboxStart,cameraRangeBeginCmd,cameraRangeServiceCmd,cameraRangeEndCmd);

//	onTrig(2,xboxBack,WheelOnOffBegin,WheelOnOffService,0);

//	onTrig(2,xboxY,manualSetY,0,0);
	//onTrig(2,xboxB,manualSetB,0,0);
//	onTrig(2,xboxB,ReverseWheel,0,0);
//	onTrig(2,xboxX,manualSetX,0,0);

//	onTrig(2,xboxLS,tglAppatureBegin,tglAppatureService,0);

//	onTrig(2,xboxRS,autoLoadBegin,0,autoLoadEnd);



//	onTrig(1,xboxRightStickClick,0,0,liftTglCmd );

	 //   onTrig(1,xboxLeftStickClick,tglGyroBeg,tglGyroServie,tlgGyroEnd );//tglGyroCmd
		//
	 //
	 //   onTrig(1,xboxStart    ,feederOnCmd,0,toroBlockAfterShotTimeCmd);
		//onTrig(1,xboxA        ,feederSpeedOnCmd,feederSpeedServiceCmd,feederSpeedOffCmd);
		//
		//
		////onTrig(1,xboxStart,cameraRangeBeginCmd,cameraRangeServiceCmd,cameraRangeEndCmd);
		//onTrig(1,xboxBack,WheelOnOffBegin,WheelOnOffService,0);
		//
		//onTrig(1,xboxY,manualSetY,0,0);
		////onTrig(1,xboxB,manualSetB,0,0);
		//
		//onTrig(1,xboxB,ReverseWheel,0,0);
		//onTrig(1,xboxX,manualSetX,0,0);

		//onTrig(1,xboxLS,tglAppatureBegin,tglAppatureService,0);
		//
		//onTrig(1,xboxRS,autoLoadBegin,0,autoLoadEnd);
		//
		//onKey('W',WheelOnOffBegin,WheelOnOffService,0);
	  //  onKey('U',Turn180,0,0);
	  //  onKey('R',ReverseDriveToggle,0,0);
	  //  onKey('S',Spin180,0,0);

  /*      onHatX( 2, 1,Spin180           ,0,0);
		onHatX( 2,-1,ReverseDriveToggleNoHold,0,0);
 */

 //onTrig(2,xboxRS,tglCamera,0,0);

//onTrig(1,xboxY,0,0,tglAppatureFlagTop);
//onTrig(1,xboxB,0,0,tglAppatureFlagMid);
//onTrig(1,xboxA,0,0,tglAppatureFlagBot);




//onTrig(2,xboxY,fullStop,0,0);

//--------------------- JoyStick One --------------------

/*
onTrig(1,xboxRS,FireTheBallCmd,0,FireTheBallCmdEnd  );
onTrig(1,xboxLS,spitBallsBeg,spitBallsService,CollectBallsTgl );

//onTrig(1,xboxLeftStickClick,0,0,tglShiftModeCmd );

onTrig(1,xboxA,manualSetA,manualSetService,manualSetEnd);
onTrig(1,xboxB,manualSetB,manualSetService,manualSetEnd);
onTrig(1,xboxX,manualSetX,manualSetService,manualSetEnd);
onTrig(1,xboxY,manualSetY,manualSetService,manualSetEnd);

onTrig(1,xboxBack,0,0 ,driverSelectCmd);

onTrig(1,xboxRightStickClick  ,toggleAutoTracking,0,0);

onTrig(1,xboxStart  ,toggleTurretTracking,0,0);

*/

//onTrig(2,xboxRS,WireUpBeginCmd,0,0);
//onTrig(2,xboxLS,WireDnBeginCmd,0,0);
/*
	onTrig(2,xboxLeftStickClick,0,0,tglShiftModeCmd );

	onTrig(2,xboxA    ,ShotACmd ,0,0);
	onTrig(2,xboxB    ,ShotBCmd ,0,0);
	onTrig(2,xboxX    ,ShotXCmd ,0,0);
	onTrig(2,xboxY    ,ShotYCmd ,0,0);
	onTrig(2,xboxRS   ,ShotRSCmd,0,0);

	onHatX( 2,-1,ShotLHCmd,0,0);
	onHatX( 2, 1,ShotRHCmd,0,0);
*/
//onTrig(2,xboxRS,ReverseDriveCmd,0,ForwardDriveCmd);

//onTrig(2,xboxRS,toggleArmManualCmd,0,0);

//onTrig(2,xboxLS,toggleToroUpDown,toggleToroUpDownService,toggleToroUpDownFinish);

//onTrig(2,xboxStart  ,ToggleBumperUpCmd,0,0);

//onTrig(2,xboxBack,ToggleCatcherOpenCloseCmd,0,0 );

//onTrig(2,xboxRightStickClick  ,toggleArmManualCmd,0,0);


//---------------------------


	//onTrig(1,xboxLeftStickClick,0,0,tglShiftModeCmd );

	//onTrig(1,xboxA    ,ShotACmd ,0,0);
	//onTrig(1,xboxB    ,ShotBCmd ,0,0);
	//onTrig(1,xboxX    ,ShotXCmd ,0,0);
	//onTrig(1,xboxY    ,ShotYCmd ,0,0);
	//onTrig(1,xboxRS   ,ShotRSCmd,0,0);

	//onTrig(2,xboxRS,ReverseDriveCmd,0,ForwardDriveCmd);

	//onTrig(2,xboxRS,toggleArmManualCmd,0,0);

	/*

	onTrig(1,xboxLS,toggleToroUpDown,toggleToroUpDownService,toggleToroUpDownFinish);

	onTrig(1,xboxStart  ,ToggleBumperUpCmd,0,0);

	onTrig(1,xboxBack,ToggleCatcherOpenCloseCmd,0,0 );

	onTrig(1,xboxRightStickClick  ,toggleArmManualCmd,0,0);




	onTrig(3,xboxLeftStickClick,0,0,tglShiftModeCmd );

	onTrig(3,xboxA    ,ShotACmd ,0,0);
	onTrig(3,xboxB    ,ShotBCmd ,0,0);
	onTrig(3,xboxX    ,ShotXCmd ,0,0);
	onTrig(3,xboxY    ,ShotYCmd ,0,0);
	onTrig(3,xboxRS   ,ShotRSCmd,0,0);

	onHatX( 3,-1,ShotLHCmd,0,0);
	onHatX( 3, 1,ShotRHCmd,0,0);
*/
//onTrig(2,xboxRS,ReverseDriveCmd,0,ForwardDriveCmd);

//onTrig(2,xboxRS,toggleArmManualCmd,0,0);

//onTrig(3,xboxLS,toggleToroUpDown,toggleToroUpDownService,toggleToroUpDownFinish);

//onTrig(3,xboxStart  ,ToggleBumperUpCmd,0,0);

//onTrig(3,xboxBack,ToggleCatcherOpenCloseCmd,0,0 );

//onTrig(3,xboxRightStickClick  ,toggleArmManualCmd,0,0);


//onTrig(2,xboxA,ButtonTest,0,0);

	void aton1();

	//void GrabIt();
   // void UnGrabIt();
   // void StopGrabIt();

	void CloseJaws();
	void OpenJaws();

	//onTrig(2,xboxA,aton1,0,0);

	void Suck();
	void Spit();
	void SuckStop();

	// onTrig(2,xboxX,Suck,0,0);

	// onTrig(2,xboxA,Spit,0,0);

	 //onTrig(2,xboxB,SuckStop,0,0);

	//onTrig(1, xboxA, CloseJaws, 0, 0);

	//onTrig(1, xboxB, OpenJaws, 0, 0);

	void TotesDown();
	void TotesUp();
	void TotesStop();

	void SuckStart();
	void SuckService();
	void SuckEnd();

	//onTrig(2, xboxLS, Spit, 0, SuckStop);
	//onTrig(2, xboxRS, SuckStart, SuckService, SuckEnd);

	void LoadOnceHigh();
	void LoadOnceLow();

	//onTrig(2, xboxY, LoadOnceHigh, 0, 0);
	//onTrig(2, xboxA, LoadOnceLow, 0, 0);

	void StoreArm();
	void ArmService();
	void StopArm();


	//onTrig(1, xboxRS, StoreArm, ArmService, StopArm);

	void StartingArm();


	//onTrig(1, xboxStart, StartingArm, ArmService, StopArm);

	//onTrig(2, xboxBack, StartingArm, ArmService, StopArm);

	void SuckFast();

	//onTrig(2, xboxStart, SuckFast, 0, 0);

	void topLoadArm();
	void topLoadArmService();

	//  onTrig(1,xboxLS,topLoadArm,topLoadArmService,StopArm);

	void binLoadArm();

	// onTrig(1, xboxLS, binLoadArm, ArmService, StopArm);

	void midLoadArm();

	//onTrig(1, xboxLS, binLoadArm, ArmService, StopArm);


	void StopAllThings();

	void GyroOff();
	void alignSpeed();
	void alignStop();

	// onTrig(1,xboxBack,StopAllThings,0,0 );
	// onTrig(2,xboxX,alignSpeed,GyroOff,alignStop );

	void LongArm();

	//onTrig(1, xboxBack, LongArm, ArmService, StopArm);

	void SpitRight();
	void SpitLeft();

	//onTrig(2, xboxB, SpitRight, 0, 0);
	//onTrig(2, xboxX, SpitLeft, 0, 0);

	void SpitRightForce();
	void SpitLeftForce();


	//onTrig(1, xboxY, 0, SpitRightForce, SuckStop);
	//onTrig(1, xboxX, 0, SpitLeftForce, SuckStop);

	void aton2();

	//  onTrig(1,xboxLeftStickClick,0,0,aton2 );

	void CopAction();

	// onTrig(2,xboxBack,CopAction,0,0);

	void BinToCenter();

	// onTrig(1,xboxY,BinToCenter,0,StopArm);

	void HighPickup();
	void LowPickup();

	//onTrig(2, xboxY, HighPickup, 0, 0);
	//onTrig(2, xboxA, LowPickup, 0, 0);


	printf("DriveLoop\n");

}


float scaleR = 0.7;

float tAccel = 2000;


void MoveTestUp()
{

	//tMoveP( 31+2, 500, 999,0,20);

	tMoveP(31 + 2, 300 * scaleR, tAccel*scaleR, 0, 20);

}

void poundIt();

void HighPickup()
{

	//tMoveP( 0, 150, 500,poundIt,4);

	if (getToteLiftAct() < 12) { MoveTestUp(); return; }

	tMoveP(17, 300 * scaleR, tAccel*scaleR, poundIt, 4);

}

void poundIt()
{

	//tMoveP( -2, 100, 999,0,4);

	//tMoveP( -2, 300, 1000,0,4);

	tMoveP(-8, 300 * scaleR, tAccel*scaleR, MoveTestUp, 4);


}

void MoveLevelOne()
{

	tMoveP(17, 300 * scaleR, tAccel*scaleR, 0, 4);

}

void LowPickup();

void MoveLevelOneHigh()
{

	tMoveP(17, 300 * scaleR, tAccel*scaleR, LowPickup, 4);

}
void LowPickup()
{

	if (getToteLiftAct() < 6) { MoveLevelOne(); return; }

	if (getToteLiftAct() > 18) { MoveLevelOneHigh(); return; }

	tMoveP(-8, 300 * scaleR, tAccel*scaleR, MoveLevelOne, 4);

}



int testFlag = 0;
int testCount = 0;

void testTable()
{

	testFlag = 0;
	testCount = 0;

	tableSize = 0;

	testFlag = 1;




	//tprintf("Table Test\n");
	//
	//tableSize=200;

	//for (int L=0; L<200; L++)
	//{
	//	table1[L]=L;
	//	table2[L]=L & 0xF0;
	//	table3[L]=400+L;

	//}



}


