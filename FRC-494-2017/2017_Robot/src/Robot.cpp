#include "WPILib.h"
#include "CANTalon.h"

#include "Timer.h"
#include "base.h"

#include "ADIS16448_IMU.h"




int teamNum = 70;

float cam1DriveMode = 1;


int inAtonRotate = 0;


float curTime2;
float lastTime2=0;

extern float PegGoalAngle;

void UpdateEncoders();
float lastSetPos = 0;
//#include <taskLib.h>
void aton_move(float speed, float strafeSpeed, float time, float x=0);
extern float inAton;
extern float inTele;
extern float inDisable;
float hopperSpeed = 0;
extern float sendBackValues[420];

void ReadEncoderSlow();

float currentHopperPOS;

int joyOverride = 0;
float joyXOverride = 0;
float joyYOverride = 0;
float joyZOverride = 0;

void Set_Winch(float speed);
void SetAnalogEncoders();

int inTest = 0;

int leftFrontDrive1 = 16;
int leftFrontDrive2 = 1;
int rightFrontDrive1 = 15;
int rightFrontDrive2 = 14;
int leftBackDrive1 = 2;
int leftBackDrive2 = 3;
int rightBackDrive1 = 13;
int rightBackDrive2 = 12;

int hopperMotor = 4;
int shooterMotor = 5;
int winchMotor1 = 6;
int winchMotor2 = 7;
int intakeRoller = 11;
int toolbarMotor = 10;
int gearLeftRoller = 9;
int gearRightRoller = 8;

int turret = 17;

float sendVars[400];

void IFC_Local_Loop_Service(void);
void UpdateVariables();



void UDCReceiverTest();
void UDCReceiverTestfrompi();
int sendUDC(char *buf, int size);
int sendUDCtoPI(char *buf, int size);

void  customJoySticks();

void setupCANTalons();

/*
CANTalon * hopperCAN;
CANTalon * shooterCAN;
CANTalon * winch1CAN;
CANTalon * winch2CAN;
CANTalon * intakeRollerCAN;
CANTalon * toolbarCAN;
CANTalon * gearRightRollerCAN;
CANTalon * gearLeftRollerCAN;

CANTalon * rightFrontDrive1Motor;
CANTalon * rightFrontDrive2Motor;
CANTalon * leftFrontDrive1Motor;
CANTalon * leftFrontDrive2Motor;
CANTalon * rightBackDrive1Motor;
CANTalon * rightBackDrive2Motor;
CANTalon * leftBackDrive1Motor;
CANTalon * leftBackDrive2Motor;
*/

Joystick *m_driveStick;
Joystick *m_driveStick2;


float  shooterLastSpeed = 0;
double shooterLastSpeedTime = 0;

//Servo hangServo;


ADIS16448_IMU *imu;

void Set_Motor(){


}


void Spit() {}
void MecanumDrive(double x, double y, double rotation, double gyroAngle);
void mainRobotLoop();

void StartNewCameraCmd()
{
	// GetMyIPAddress ();
	//	if (CameraConnectRetry+5.0>GetTime()) return;

	//#ifdef TEAM14

	  // Camera = new iAxisCamera("10.0.14.11");

	//#endif

	//#ifdef TEAM70

		//Camera = new iAxisCamera("10.0.70.11");

	//#endiff

	//#ifdef TEAM494

		//Camera = new iAxisCamera("10.4.94.11");

	//#endif

	//	Camera = new iAxisCamera(TeamCameraIP);


}

int isEnabled()
{

	return 1;

}

void MoveElev(float dest)
{
	// ElevDest=dest;
	 //ElevAuto=1;
}

void MoveShoe(float dest)
{
	// ShoeDest=dest;
	 //ShoeAuto=1;
}

void MoveToolBar(float dest)
{
	// ToolBarDest=dest;
	 //ToolBarAuto=1;
}

void SetSolenoid(int num, int state)
{

	// if (num<0 || num>7) return;

	// if (state) solenoid[num]->Set(true);

	// else

	// solenoid[num]->Set(false);

}

extern float realGyroAngle;
/*
void resetGyro()
{
	//gyroZero=realGyroAngle;
}
*/
void startTimer(uc num, int count, void(*serviceAddress)(uc), void(*finishAddress)(uc));

std::thread FastThread;

//char TeamCameraIP[20] = "10.4.94.11";

char TeamDriveStationIP[20] = "10.0.14.5";
char TeamPiIP[20] = "10.0.14.100";




int GetMyIPAddress();

float realGyroAngle = 0;
extern float UseGyro;

extern int gyroRotateFlag;

#define noCan1X
#define noCan2

#define max(x, y) (((x) > (y)) ? (x) : (y))

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.





#define  LIDAR_ADDR 0x62
#define LIDAR_CONFIG_REGISTER  0x00
#define LIDAR_DISTANCE_REGISTER  0x8f

extern float blockCanIO;

//extern DigitalOutput *lidarSelectPtr1;
//extern DigitalOutput *lidarSelectPtr2;

extern float lidarValue;
extern float jawsClosed;

//extern float newImageCount;

//DriverStation *m_ds;

//extern float inDisable;

Solenoid *solenoid[20];

int operatorOnce = 0;

float gyroZero = 0;

float gyroAngle;

int gyroZeroFlag = 0;

int gTimeOut = 0;


void GyroOff()
{

	//gyroZeroFlag = 1;
}

int initEncoderTask();

void setLidarSelect(float var);

float getToteLiftAct();
float getBinLiftAct();
void zeroToteLift();
void zeroBinLift();


//void BinArmAzm         (float speed);
//void BinJaw            (float speed);
//void BinArmElbow       (float speed);
//void BinArmWrist       (float speed);
//void ToteForkRetract   (float speed);
//void ToteForkIntake    (float speed);

//void ToteLift (float speed);
//void BinLift  (float speed);


//void Suck();
//void Spit ();
//void SuckStop();


int bActive = 0;
int tActive = 0;
int aActive = 0;
int eActive = 0;
int sActive = 0;

extern float tDisable;
extern float bDisable;
extern float aDisable;
extern float eDisable;
//  extern float sDisable;


Joystick *jStick1 = 0;
Joystick *jStick2 = 0;
//Joystick *jStick3=0;
//Joystick *jStick4=0;

float autoDriveY = 0.0;
float autoDriveX = 0.0;
float autoDriveR = 0.0;

int scanFlag = 0;

void userNewDriverStationPacket();
void userInit();
void userFastLoop(void);

static float fabs(float val)
{
	if (val >= 0) return val;

	return -val;
}

int isEnabled();

bool IsDisabled()
{

	if (isEnabled()) return false;

	return true;

}

void ShootBalls2();
float getToteLiftAct()
{

#ifndef noCan2

	if (blockCanIO == 1 || blockCanIO == 2) { return 0; }

	float pos = -mToteLift1->GetPosition() / 200.0 - tZero;

	//loc[pwmToteLift1]=pos;

	return pos;

#endif

#ifdef noCan

	return 0;

#endif

	return 0;

}

void zeroToteLift()
{

#ifndef noCan2

	if (blockCanIO == 1 || blockCanIO == 2) { tZero = 0; return; }

	tZero = -mToteLift1->GetPosition() / 200.0;

	tCmdPos = 0;

#endif

}

float getBinLiftAct()
{
#ifndef noCan2

	if (blockCanIO == 1 || blockCanIO == 2) { return 0; }

	float pos = mBinLift1->GetPosition() / 200.0 - bZero;

	//loc[pwmBinLift1]=pos;

	return pos;

#endif

#ifdef noCan

	return 0;

#endif

	return 0;

}

void zeroBinLift()
{
#ifndef noCan2

	if (blockCanIO == 1 || blockCanIO == 2) { bZero = 0; return; }

	bZero = mBinLift1->GetPosition() / 200.0;

	bCmdPos = 0;

#endif

}

void GrabIt()
{
	//	BinJaw (1);
}

void UnGrabIt()
{
	//BinJaw (-1);
}

void StopGrabIt()
{
	//BinJaw (0);

}

void StopGrabIt(unsigned char i)
{
	//BinJaw (0);

}

void CloseJaws()
{
	GrabIt();

	jawsClosed = 1;

	//ttprintf("CloseJaws\n");

	startTimer(10, 100, 0, StopGrabIt);
}

void OpenJaws()
{
	UnGrabIt();

	jawsClosed = 0;

	//ttprintf("OpenJaws\n");

	startTimer(10, 100, 0, StopGrabIt);
}
void threadTest()
{

	//printf("ThreadTest\n");

loop:

	goto loop;


}


class MecanumDefaultCode : public IterativeRobot
{
	//CANTalon lf; /*left front */
	//CANTalon lr;/*left rear */
	//CANTalon rf; /*right front */
	//CANTalon rr; /*right rear */

	CANTalon * _talon20 = new CANTalon(20);
	CANTalon * _talon21 = new CANTalon(21);
	CANTalon * _talon22 = new CANTalon(22);
	CANTalon * _talon23 = new CANTalon(23);





public:

	AnalogInput *ai = new AnalogInput(0);
	Potentiometer * pot = new AnalogPotentiometer(ai, 10000, 30);
	Servo *antibackdrive = new Servo(1);

		CANTalon * hopperCAN = new CANTalon(hopperMotor);
		CANTalon * shooterCAN = new CANTalon(shooterMotor);
		CANTalon * winch1CAN = new CANTalon(winchMotor1);
		CANTalon * winch2CAN = new CANTalon(winchMotor2);
		CANTalon * intakeRollerCAN = new CANTalon(intakeRoller);
		CANTalon * toolbarCAN = new CANTalon(toolbarMotor);
		CANTalon * gearRightRollerCAN = new CANTalon(gearRightRoller);
		CANTalon * gearLeftRollerCAN = new CANTalon(gearLeftRoller);
		CANTalon * turretCAN = new CANTalon(turret);

	    CANTalon * rightFrontDrive1Motor = new CANTalon(rightFrontDrive1);
		CANTalon * rightFrontDrive2Motor = new CANTalon(rightFrontDrive2);
		CANTalon * leftFrontDrive1Motor = new CANTalon(leftFrontDrive1);
		CANTalon * leftFrontDrive2Motor = new CANTalon(leftFrontDrive2);
		CANTalon * rightBackDrive1Motor = new CANTalon(rightBackDrive1);
		CANTalon * rightBackDrive2Motor = new CANTalon(rightBackDrive2);
		CANTalon * leftBackDrive1Motor = new CANTalon(leftBackDrive1);
		CANTalon * leftBackDrive2Motor = new CANTalon(leftBackDrive2);
	//RobotDrive *m_robotDrive;		// RobotDrive object using PWM 1-4 for drive motors
	Joystick *m_driveStick;			// Joystick object on USB port 1 (mecanum drive)public:
	Joystick *m_shootStick;
	Joystick *m_driveStick2;
	//AnalogGyro gyro;
	/**
	 * Constructor for this "MecanumDefaultCode" Class.
	 */




	MecanumDefaultCode(void)   //: //gyro(0) //lf(20), lr(21), rf(22), rr(23), gyro(0)
	{

		imu = new ADIS16448_IMU;
		printf("MecanumDefaultCode\n");
		userInit();
		/*
		rightFrontDrive1Motor->SetControlMode(CANSpeedController::kPercentVbus);
		rightFrontDrive2Motor->SetControlMode(CANSpeedController::kPercentVbus);
		leftFrontDrive1Motor ->SetControlMode(CANSpeedController::kPercentVbus);
		leftFrontDrive2Motor ->SetControlMode(CANSpeedController::kPercentVbus);
		rightBackDrive1Motor ->SetControlMode(CANSpeedController::kPercentVbus);
		rightBackDrive2Motor ->SetControlMode(CANSpeedController::kPercentVbus);
		leftBackDrive1Motor  ->SetControlMode(CANSpeedController::kPercentVbus);
		leftBackDrive2Motor  ->SetControlMode(CANSpeedController::kPercentVbus);
*/
/*
			int absolutePosition = rightFrontDrive1Motor->GetPulseWidthPosition() & 0xFFF;
			rightFrontDrive1Motor->SetEncPosition(absolutePosition);
			rightFrontDrive1Motor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
			rightFrontDrive1Motor->SetSensorDirection(false);
			rightFrontDrive1Motor->ConfigNominalOutputVoltage(+0., -0.);
			rightFrontDrive1Motor->ConfigPeakOutputVoltage(+12., -12.);
			rightFrontDrive1Motor->SetAllowableClosedLoopErr(0);
			rightFrontDrive1Motor->SelectProfileSlot(0);
			rightFrontDrive1Motor->SetF(0.0);
			rightFrontDrive1Motor->SetP(0.1);
			rightFrontDrive1Motor->SetI(0.0);
			rightFrontDrive1Motor->SetD(0.0);
			rightFrontDrive1Motor->SetControlMode(CANSpeedController::kPercentVbus);
			absolutePosition = rightFrontDrive1Motor->GetPulseWidthPosition() & 0xFFF;
			rightFrontDrive2Motor->SetEncPosition(absolutePosition);
			rightFrontDrive2Motor->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
			rightFrontDrive2Motor->SetSensorDirection(false);
			rightFrontDrive2Motor->ConfigNominalOutputVoltage(+0., -0.);
			rightFrontDrive2Motor->ConfigPeakOutputVoltage(+12., -12.);
			rightFrontDrive2Motor->SetAllowableClosedLoopErr(0);
			rightFrontDrive2Motor->SelectProfileSlot(0);
			rightFrontDrive2Motor->SetF(0.0);
			rightFrontDrive2Motor->SetP(0.1);
			rightFrontDrive2Motor->SetI(0.0);
			rightFrontDrive2Motor->SetD(0.0);
			rightFrontDrive2Motor->SetControlMode(CANSpeedController::kPercentVbus);

*/

		m_driveStick = new Joystick(0);
		m_shootStick = new Joystick(1);

		/*
		int absolutePosition=0;

			 absolutePosition = _talon20->GetPulseWidthPosition() & 0xFFF;
			_talon20->SetEncPosition(absolutePosition);
 _talon20->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
			_talon20->SetSensorDirection(false);
			_talon20->ConfigNominalOutputVoltage(+0., -0.);
			_talon20->ConfigPeakOutputVoltage(+12., -12.);
			_talon20->SetAllowableClosedLoopErr(0);
			_talon20->SelectProfileSlot(0);
			_talon20->SetF(0.0);
			_talon20->SetP(0.1);
			_talon20->SetI(0.0);
			_talon20->SetD(0.0);
			_talon20->SetControlMode(CANSpeedController::kPercentVbus);

			 absolutePosition = _talon21->GetPulseWidthPosition() & 0xFFF;
				_talon21->SetEncPosition(absolutePosition);
				_talon21->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
				_talon21->SetSensorDirection(false);
				_talon21->ConfigNominalOutputVoltage(+0., -0.);
				_talon21->ConfigPeakOutputVoltage(+12., -12.);
				_talon21->SetAllowableClosedLoopErr(0);
				_talon21->SelectProfileSlot(0);
				_talon21->SetF(0.0);
				_talon21->SetP(0.1);
				_talon21->SetI(0.0);
				_talon21->SetD(0.0);

				_talon21->SetControlMode(CANSpeedController::kPercentVbus);



				 absolutePosition = _talon22->GetPulseWidthPosition() & 0xFFF;
					_talon22->SetEncPosition(absolutePosition);
					_talon22->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
					_talon22->SetSensorDirection(false);
					_talon22->ConfigNominalOutputVoltage(+0., -0.);
					_talon22->ConfigPeakOutputVoltage(+12., -12.);
					_talon22->SetAllowableClosedLoopErr(0);
					_talon22->SelectProfileSlot(0);
					_talon22->SetF(0.0);
					_talon22->SetP(0.1);
					_talon22->SetI(0.0);
					_talon22->SetD(0.0);

					_talon22->SetControlMode(CANSpeedController::kPercentVbus);

					 absolutePosition = _talon23->GetPulseWidthPosition() & 0xFFF;
						_talon23->SetEncPosition(absolutePosition);
						_talon23->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
						_talon23->SetSensorDirection(false);
						_talon23->ConfigNominalOutputVoltage(+0., -0.);
						_talon23->ConfigPeakOutputVoltage(+12., -12.);
						_talon23->SetAllowableClosedLoopErr(0);
						_talon23->SelectProfileSlot(0);
						_talon23->SetF(0.0);
						_talon23->SetP(0.1);
						_talon23->SetI(0.0);
						_talon23->SetD(0.0);

						_talon23->SetControlMode(CANSpeedController::kPercentVbus);

*/



	}
	void RotateVector(double &x, double &y, double angle)
	{
		double cosA = cos(angle * (3.14159 / 180.0));
		double sinA = sin(angle * (3.14159 / 180.0));
		double xOut = x * cosA - y * sinA;
		double yOut = x * sinA + y * cosA;
		x = xOut;
		y = yOut;
	}
	void Normalize(double *wheelSpeeds)
	{
		double maxMagnitude = fabs(wheelSpeeds[0]);
		int32_t i;
		for (i = 1; i < 4; i++)
		{
			double temp = fabs(wheelSpeeds[i]);
			if (temp > maxMagnitude) maxMagnitude = temp;
		}
		if (maxMagnitude > 1.0)
		{
			for (i = 0; i < 4; i++)
			{
				wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
			}
		}
	}


	//I2C *lidar = new I2C(I2C::Port::kOnboard, 0x62);
	/*
	void readyUp()
	{
		const int tries = 5;
		int ret = 1;
		for (int i = 0; ret != 0 && i < tries; i++)
		{
			ret = lidar->Write(0x00, 0x04);

			::Wait(0.004);
		}
	}

	double getDistance() //returns distance in centimeters
	{
		const int tries = 10;
		int ret = 1;

		readyUp();
		unsigned char *distArray = new unsigned char[8];
		for (int i = 0; ret != 0 && i < tries; i++)
		{
			::Wait(0.003);
			ret = lidar->Read(0x8f, 2, distArray);
			//printf("Success? %d\n", ret);
		}

		int centimeters = (distArray[0] << 8) + distArray[1];

		delete[] distArray;

		return centimeters;
		return 0;
	}
	*/

	float vars[2000];
	void  mainRobotLoop()
	{
		//printf("peggoalangle: %f\n", PegGoalAngle);
		//SetAnalogEncoders();
		//UpdateEncoders();

		//printf("%f\n", getDistance());

		vars[0] = selectCamera;
		vars[1] = cam1dr;
		vars[2] = cam1dg;
		vars[3] = cam2dr;
		vars[4] = cam2dg;
		vars[5] = cam1DriveMode;
		sendUDCtoPI((char*)vars,8000);

		customJoySticks();

		userNewDriverStationPacket();

		IFC_Local_Loop_Service();


		UDCReceiverTest();

		UDCReceiverTestfrompi();

		UpdateVariables();

		GlobalSendBack();

		char *ptr=(char*)&sendVars[0];

		ptr[0]=1;
		ptr[1]=19;
		ptr[2]=2;
		ptr[3]=108;

		for(int i=1;i<400;i++){
			sendVars[i] = sendBackValues[i+1];

		}

		//sendVars[300]=1.234;



		sendUDC((char*)sendVars, 1600);
		//sendUDC("supe",5);

		static double lastTime3 = 0;

		double curTime = frc::GetClock();

		double dif2 = curTime - lastTime3;

		if (dif2 < 0.01) return;

		lastTime3 = curTime;

		double gyroX = imu->GetAngleX();

		double gyroY = imu->GetAngleY() / 3.6;

		double gyroZ = imu->GetAngleZ();

		double joyXX = m_driveStick->GetRawAxis(0); //if ((joyX)<0.01) joyX=0; if (joyX>limit) joyX=limit;
		double joyYY = m_driveStick->GetRawAxis(1); //if ((joyY)<0.01) joyY=0; if (joyY>limit) joyY=limit;
		double joyZZ = m_driveStick->GetRawAxis(4); //if ((joyZ)<0.01) joyZ=0; if (joyZ>limit) joyZ=limit;

		if(joyXX+joyYY+joyZZ>0.2)
				{
					joyOverride=0;
				}
		if(joyOverride){
			printf("OVERRIDE %3.3f %3.3f %3.3f\n",  joyXOverride,joyYOverride,joyZOverride);
			joyX[2] = joyXOverride;
			joyY[2] = joyYOverride;
			joyX2[2] = joyZOverride;
		}
		else{
					joyX[2] = joyXX;
					joyY[2] = joyYY;
					joyX2[2] = joyZZ;
				}




		gyroAngle = -gyroX;

		float x = joyX[2]; if (x<0.05 && x>-0.05) x = 0;
		float y = joyY[2]; if (y<0.05 && y>-0.05) y = 0;
		float r = joyX2[2]; if (r<0.05 && r>-0.05) r = 0;

		static float lastx, lasty, lastr;
		static int gTimeOut = 0;
		static double releaseTime = 0;

		if (x == lastx && y == lasty && r == lastr && inAton==0)
		{
			gTimeOut++;

			if (gTimeOut > 500) gyroZero = gyroAngle;

		}
		else
			gTimeOut = 0;

		lastx = x;
		lasty = y;
		lastr = r;


		if (gyroZeroFlag && inAton==0) { gyroZero = gyroAngle; gyroZeroFlag = 0; }

		if (fabs(r) > 0.2 && inAton==0)
		{
			releaseTime = frc::GetClock() + 0.5;

			gyroZero = gyroAngle;
		}

		if (frc::GetClock() < releaseTime && inAton==0)
		{
			gyroZero = gyroAngle;
		}




		x = x*x*x;


		y = y*y*y;

		r *= 0.8;


		r = r*r*r;

		float gyroAdj = (gyroAngle - gyroZero) / 18 / 2 / 1.5/5;
		if(inAtonRotate){
			gyroAdj=0;
		}

		r = r - gyroAdj;

		//y -= autoDriveY;
		//x += autoDriveX;

		// robotDrive.MecanumDrive_Cartesian(x,y,r);

		double xIn = x;
		double yIn = y;
		double rotation = r;

		yIn = -yIn;

		RotateVector(xIn, yIn, 0);

		gyro_Zero=gyroZero;
		GyroLoc=gyroAngle-gyroZero;
		gyro_Adjust = gyroAdj;

		double wheelSpeeds[4];
		wheelSpeeds[1] = -(xIn + yIn + rotation);
		wheelSpeeds[2] = (-xIn + yIn - rotation);
		wheelSpeeds[3] = -(-xIn + yIn + rotation);
		wheelSpeeds[4] = (xIn + yIn - rotation);

		Normalize(wheelSpeeds);


		leftFrontDrive1Motor->Set(wheelSpeeds[1]);
		leftFrontDrive2Motor->Set(wheelSpeeds[1]);
		rightFrontDrive1Motor->Set(wheelSpeeds[2]);
		rightFrontDrive2Motor->Set(wheelSpeeds[2]);
		leftBackDrive1Motor->Set(wheelSpeeds[3]);
		leftBackDrive2Motor->Set(wheelSpeeds[3]);
		rightBackDrive1Motor->Set(wheelSpeeds[4]);
		rightBackDrive2Motor->Set(wheelSpeeds[4]);




	}


	void RobotInit() { printf("RobotInit\n"); setupCANTalons();
	if (teamNum==70){
		strcpy(TeamDriveStationIP, "10.0.70.5");
		strcpy(TeamPiIP, "10.0.70.199");
	}
	if (teamNum== 494){
		strcpy(TeamDriveStationIP, "10.4.94.5");
		strcpy(TeamPiIP, "10.4.94.199");
	}

	if (teamNum== 14){

		strcpy(TeamDriveStationIP, "10.0.14.5");
		strcpy(TeamPiIP, "10.0.14.100");

	}

	}
	void zeroGyro(){
		gyroZero=gyroAngle;
	}
	void DisabledInit() { printf("DisableInit\n"); mainRobotLoop();}
	void DisabledPeriodic() { inDisable = 1; zeroGyro(); inTest = 0; inTele = 0; inAton = 0; inAtonRotate=0; mainRobotLoop();}
	void RobotPeriodic() { mainRobotLoop(); }
	void TeleopInit() { joyOverride = 0; printf("TeleopInit\n"); mainRobotLoop();}
	void AutonomousInit() { printf("AutomousInit\n"); mainRobotLoop();}
	void AutonomousPeriodic() { inDisable = 0; inTest = 0; inTele = 0; inAton = 1;  mainRobotLoop(); }
	void TestInit() { printf("TestInit\n"); mainRobotLoop();}
	void TestPeriodic() { inDisable = 0; inTest = 1; inTele = 0; inAton = 0; inAtonRotate=0;  mainRobotLoop();}
	//void StartCompetition  (){printf("StartCompetition\n");}


	/** @return 10% deadband */
	double Db(double axisVal)
	{
		if (axisVal < -0.10)
			return axisVal;
		if (axisVal > +0.10)
			return axisVal;
		return 0;
	}
	/**
	 * Gets called once for each new packet from the DS.
	 */
	void TeleopPeriodic(void)
	{
		inAtonRotate=0;
		mainRobotLoop();
	}
};

void MecanumDrive(double x, double y, double rotation, double gyroAngle) {


}






/*
void Set_f(float speed) {}; // { if (blockCanIO != 1) { FrontDriveRt_Can      ->Set(speed); } pwm[ FrontDriveRt_Num     ] = speed; };
void Set_RearDriveRt(float speed) {}; // { if (blockCanIO != 1) { RearDriveRt_Can       ->Set(speed); } pwm[ RearDriveRt_Num		] = speed; };
void Set_WinchMotor1(float speed) {}; // { if (blockCanIO != 1) { WinchMotor1_Can		  ->Set(speed); } pwm[ WinchMotor1_Num      ] = speed; };
void Set_ToolBar(float speed) {}; // { if (blockCanIO != 1) { ToolBar_Can			  ->Set(speed); } pwm[ ToolBar_Num 			] = speed; };
//void Set_HeadLights			(float speed)  { if (blockCanIO != 1) { HeadLights_Can		  ->Set(speed); } pwm[ HeadLights_Num 		] = speed; };
void Set_ShooterElevation(float speed) {}; // { if (blockCanIO != 1) { ShooterElevation_Can  ->Set(speed); } pwm[ ShooterElevation_Num	] = speed; };
void Set_ToroRight(float speed) {}; // { if (blockCanIO != 1) { ToroRight_Can		  ->Set(speed); } pwm[ ToroRight_Num 		] = speed; };
void Set_BallFeed(float speed) {}; // { if (blockCanIO != 1) { BallFeed_Can		  ->Set(speed); } pwm[ BallFeed_Num 		] = speed; };
void Set_SpareTalon1(float speed) {}; // { if (blockCanIO != 1) { SpareTalon1_Can		  ->Set(speed); } pwm[ SpareTalon1_Num 		] = speed; };
void Set_ToroLeft(float speed) {}; // { if (blockCanIO != 1) { ToroLeft_Can		  ->Set(-speed); } pwm[ ToroLeft_Num 		] = speed; };
void Set_ShooterBallShoe(float speed) {}; // {

//	if (speed>0.5) speed=0.5;
//	if (blockCanIO != 1) { ShooterBallShoe_Can	  ->Set(speed); } pwm[ ShooterBallShoe_Num 	] = speed; };


//void Set_ShareTalon2		(float speed)  { if (blockCanIO != 1) { ShareTalon2_Can		  ->Set(speed); } pwm[ ShareTalon2_Num 		] = speed; };
void
Wheel(float speed) {}; // { if (blockCanIO != 1) { ShooterWheel_Can	  ->Set(-speed); if (shooterLastSpeed!=speed){ shooterLastSpeed=speed; shooterLastSpeedTime=GetClock();} } pwm[ ShooterWheel_Num 	] = speed; };
void Set_WinchMotor2(float speed) {}; // { if (blockCanIO != 1) { WinchMotor2_Can		  ->Set(speed); } pwm[ WinchMotor2_Num 		] = speed; };
void Set_RearDriveLt(float speed) {}; // { if (blockCanIO != 1) { RearDriveLt_Can		  ->Set(speed); } pwm[ RearDriveLt_Num 		] = speed; };
void Set_FrontDriveLt(float speed) {}; // { if (blockCanIO != 1) { FrontDriveLt_Can	  ->Set(speed); } pwm[ FrontDriveLt_Num 	] = speed; };





double Get_Speed_FrontDriveRt() { return 0; } // { if (blockCanIO != 1) { return FrontDriveRt_Can      ->GetSpeed(); } return 0; };
double Get_Speed_RearDriveRt() { return 0; } //{ if (blockCanIO != 1) { return RearDriveRt_Can       ->GetSpeed(); } return 0; };
double Get_Speed_WinchMotor1() { return 0; } //{ if (blockCanIO != 1) { return WinchMotor1_Can		  ->GetSpeed(); } return 0; };
double Get_Speed_ToolBar() { return 0; } //{ if (blockCanIO != 1) { return ToolBar_Can			  ->GetSpeed(); } return 0; };
//double Get_Speed_HeadLights		    ()  { if (blockCanIO != 1) { return HeadLights_Can		  ->GetSpeed(); } return 0; };
double Get_Speed_ShooterElevation() { return 0; } //{ if (blockCanIO != 1) { return ShooterElevation_Can  ->GetSpeed(); } return 0; };
double Get_Speed_ToroRight() { return 0; } //{ if (blockCanIO != 1) { return ToroRight_Can		  ->GetSpeed(); } return 0; };
double Get_Speed_BallFeed() { return 0; } //{ if (blockCanIO != 1) { return BallFeed_Can		  ->GetSpeed(); } return 0; };
double Get_Speed_SpareTalon1() { return 0; } //{ if (blockCanIO != 1) { return SpareTalon1_Can		  ->GetSpeed(); } return 0; };
double Get_Speed_ToroLeft() { return 0; } //{ if (blockCanIO != 1) { return ToroLeft_Can		  ->GetSpeed(); } return 0; };
double Get_Speed_ShooterBallShoe() { return 0; } //{ if (blockCanIO != 1) { return ShooterBallShoe_Can	  ->GetSpeed(); } return 0; };
double Get_Speed_ShareTalon2() { return 0; } //{ if (blockCanIO != 1) { return ShareTalon2_Can		  ->GetSpeed(); } return 0; };
double Get_Speed_ShooterWheel() { return 0; } //{ if (blockCanIO != 1) { return ShooterWheel_Can	  ->GetSpeed(); } return 0; };
double Get_Speed_WinchMotor2() { return 0; } //{ if (blockCanIO != 1) { return WinchMotor2_Can		  ->GetSpeed(); } return 0; };
double Get_Speed_RearDriveLt() { return 0; } //{ if (blockCanIO != 1) { return RearDriveLt_Can		  ->GetSpeed(); } return 0; };
double Get_Speed_FrontDriveLt() { return 0; } //{ if (blockCanIO != 1) { return FrontDriveLt_Can	  ->GetSpeed(); } return 0; };





double Get_Loc_FrontDriveRt() { return 0; } // { if (blockCanIO != 1) { return FrontDriveRt_Can      ->GetEncPosition(); } return 0; };
double Get_Loc_RearDriveRt() { return 0; } //{ if (blockCanIO != 1) { return RearDriveRt_Can       ->GetEncPosition(); } return 0; };
double Get_Loc_WinchMotor1() { return 0; } //{ if (blockCanIO != 1) { return WinchMotor1_Can		->GetEncPosition(); } return 0; };
double Get_Loc_ToolBar() { return 0; } //{ if (blockCanIO != 1) { return ToolBar_Can			->GetEncPosition(); } return 0; };
//double Get_Loc_HeadLights		  ()  { if (blockCanIO != 1) { return HeadLights_Can		->GetEncPosition(); } return 0; };
double Get_Loc_ShooterElevation() { return 0; } //{ if (blockCanIO != 1) { return ShooterElevation_Can  ->GetEncPosition(); } return 0; };
double Get_Loc_ToroRight() { return 0; } //{ if (blockCanIO != 1) { return ToroRight_Can		    ->GetEncPosition(); } return 0; };
double Get_Loc_BallFeed() { return 0; } //{ if (blockCanIO != 1) { return BallFeed_Can		    ->GetEncPosition(); } return 0; };
double Get_Loc_SpareTalon1() { return 0; } //{ if (blockCanIO != 1) { return SpareTalon1_Can		->GetEncPosition(); } return 0; };
double Get_Loc_ToroLeft() { return 0; } //{ if (blockCanIO != 1) { return ToroLeft_Can		    ->GetEncPosition(); } return 0; };
double Get_Loc_ShooterBallShoe() { return 0; } //{ if (blockCanIO != 1) { return ShooterBallShoe_Can	->GetEncPosition(); } return 0; };
double Get_Loc_ShareTalon2() { return 0; } //{ if (blockCanIO != 1) { return ShareTalon2_Can		->GetEncPosition(); } return 0; };
double Get_Loc_ShooterWheel() { return 0; } //{ if (blockCanIO != 1) { return ShooterWheel_Can	    ->GetEncPosition(); } return 0; };
double Get_Loc_WinchMotor2() { return 0; } //{ if (blockCanIO != 1) { return WinchMotor2_Can		->GetEncPosition(); } return 0; };
double Get_Loc_RearDriveLt() { return 0; } //{ if (blockCanIO != 1) { return RearDriveLt_Can		->GetEncPosition(); } return 0; };
double Get_Loc_FrontDriveLt() { return 0; } //{ if (blockCanIO != 1) { return FrontDriveLt_Can	    ->GetEncPosition(); } return 0; };


*/
/*
*/

MecanumDefaultCode * robott;

#define START_ROBOT_CLASSx(_ClassName_)                                       \
  int main() {                                                               \
    if (!HAL_Initialize(0)) {                                                \
      std::cerr << "FATAL ERROR: HAL could not be initialized" << std::endl; \
      return -1;                                                             \
    }                                                                        \
    HAL_Report(HALUsageReporting::kResourceType_Language,                    \
               HALUsageReporting::kLanguage_CPlusPlus);                      \
    static _ClassName_ robot;                                                \
    robott=&robot; \
    std::printf("\n********** Robot program starting **********\n");         \
    robot.StartCompetition();                                                \
  }


START_ROBOT_CLASSx(MecanumDefaultCode);

//static MecanumDefaultCode robot;



void Set_RealWheel(float val)
{

	robott->rightFrontDrive1Motor ->Set(1);



}

void setupCANTalons(){
	printf("setting up cantalons");
	robott->toolbarCAN->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	robott->toolbarCAN->SetSensorDirection(false);
	robott->toolbarCAN->ConfigNominalOutputVoltage(+0.0, -0.0);
	robott->toolbarCAN->ConfigPeakOutputVoltage(+12.0, -12.0);
	robott->toolbarCAN->SelectProfileSlot(0);
	robott->toolbarCAN->SetF(0.0);
	robott->toolbarCAN->SetP(0.1);
	robott->toolbarCAN->SetI(0.0);
	robott->toolbarCAN->SetD(0.0);

	robott->shooterCAN->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	robott->shooterCAN->SetSensorDirection(true);
	robott->shooterCAN->ConfigNominalOutputVoltage(+0.0, -0.0);
	robott->shooterCAN->ConfigPeakOutputVoltage(+12.0, -12.0);
	robott->shooterCAN->SelectProfileSlot(0);
	robott->shooterCAN->SetF(0.04);
	robott->shooterCAN->SetP(0.1);
	robott->shooterCAN->SetI(0.0);
	robott->shooterCAN->SetD(0.0);

	robott->turretCAN->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	robott->turretCAN->SetSensorDirection(false);
	robott->turretCAN->ConfigNominalOutputVoltage(+0.0, -0.0);
	robott->turretCAN->ConfigPeakOutputVoltage(+12.0, -12.0);
	robott->turretCAN->SelectProfileSlot(0);
	robott->turretCAN->SetF(0.00);
	robott->turretCAN->SetP(0.6);
	robott->turretCAN->SetI(0.0);
	robott->turretCAN->SetD(0.0);


	robott->hopperCAN->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	robott->hopperCAN->SetSensorDirection(false);
	robott->hopperCAN->ConfigNominalOutputVoltage(+0.0, -0.0);
	robott->hopperCAN->ConfigPeakOutputVoltage(+12.0, -12.0);
	robott->hopperCAN->SelectProfileSlot(0);
	robott->hopperCAN->SetF(0.0);
	robott->hopperCAN->SetP(0.5);
	robott->hopperCAN->SetI(0.0);
	robott->hopperCAN->SetD(0.0);
	robott->hopperCAN->SetInverted(true);

	if(teamNum==494){
		robott->rightFrontDrive2Motor->SetSensorDirection(true);
		robott->shooterCAN->SetSensorDirection(false);
		robott->hopperCAN->SetSensorDirection(true);
	}
}

float wheelZero=0;

float Get_ShooterSetPoint() {return robott->shooterCAN->GetSetpoint(); }
float Get_ShooterSpeed  () {return robott->shooterCAN   ->GetSpeed(); }
float Get_TurretPOS     () {return robott->turretCAN  ->  GetPosition() * TurretScale; }
float Get_HopperPOS     () {return robott->hopperCAN  ->  GetPosition(); }
float Get_HopperSetPoint() {return robott->hopperCAN->GetSetpoint(); }
float Get_WheelLocation() {return robott->rightFrontDrive2Motor->GetPosition()-wheelZero; }


float Get_AntiBackDrivePOS () {return robott->antibackdrive->GetPosition();}

void Set_BallIntake     (float speed) {  Set_Winch(speed);}
void Set_GearIntakeRight(float speed) {  robott->gearRightRollerCAN->SetControlMode(CANSpeedController::kPercentVbus);robott->gearRightRollerCAN->Set(speed); }
void Set_GearIntakeLeft (float speed) {  robott->gearLeftRollerCAN ->SetControlMode(CANSpeedController::kPercentVbus);robott->gearLeftRollerCAN->Set(speed); }
void Set_Winch1         (float speed) {  robott->winch1CAN         ->SetControlMode(CANSpeedController::kPercentVbus); robott->winch1CAN->Set(speed); }
void Set_Winch2         (float speed) {  robott->winch2CAN		   ->SetControlMode(CANSpeedController::kPercentVbus);robott->winch2CAN->Set(speed); }
void Set_ToolBar        (float speed) {  robott->toolbarCAN        ->SetControlMode(CANSpeedController::kPercentVbus);robott->toolbarCAN->Set(speed);}
void Set_Shooter        (float speed) {  robott->shooterCAN        ->SetControlMode(CANSpeedController::kPercentVbus);robott->shooterCAN->Set(-speed); }
void Set_Hopper         (float speed) { printf("SettingHopper\n"); robott->hopperCAN         ->SetControlMode(CANSpeedController::kPercentVbus); robott->hopperCAN->Set(speed); }
void Set_Turret         (float speed) {  robott->turretCAN         ->SetControlMode(CANSpeedController::kPercentVbus); robott->turretCAN->Set(speed); }

void Set_GearIntake		(float speed) {  Set_GearIntakeRight(-speed); Set_GearIntakeLeft(speed); }
void Set_Winch           (float speed) {printf("Setting Winch\n"); Set_Winch1(speed); Set_Winch2(-speed);}



void SetShooterToRPM(float RPM){
	robott->shooterCAN->SetControlMode(CANSpeedController::kSpeed);
	robott->shooterCAN->Set(RPM);
}
extern float ForceHopperSpeed;
void SetHopperToRPM(float RPM){
if(!ForceHopperSpeed){
	robott->hopperCAN->SetControlMode(CANSpeedController::kSpeed);
	robott->hopperCAN->Set(RPM);
}
}
void MoveHopperToPOS(float POS){
	robott->hopperCAN->SetControlMode(CANSpeedController::kPosition);
	robott->hopperCAN->Set(POS);
}

void MoveToolbarToPOS(float POS){
	robott->toolbarCAN->SetControlMode(CANSpeedController::kPosition);
	robott->toolbarCAN->Set(POS);

}

void SetAnalogEncoders(){
	//robott->hopperCAN->SetAnalogPosition(currentHopperPOS);
	ReadEncoderSlow();
	}
void MoveTurretToPOS(float position){
	robott->turretCAN->SetControlMode(CANSpeedController::kPosition);
	if(position>180){position=180;}
	if(position<0){position=0;}

	printf("MoveToolbarToPOS: %f\n", position);
	robott->turretCAN->Set(position / TurretScale);

}

void MoveToolBarToGearPickup(){
	MoveToolbarToPOS(ToolBarGearPickup);
}
void MoveToolBarToGearDropoff(){
	MoveToolbarToPOS(ToolBarGearDropoff);
}

void ManualClimbUp(){
	printf("Manual Climb UP\n");
	//Set_Winch(1);
}
void ManualClimbDown(){
	printf("Manual Climb DOWN\n");
	//Set_Winch(-1);
}
void ManualClimbStop(){
	printf("Manual Climb STOP\n");
	Set_Winch(0);
}


void MoveToolBarUp(){
	printf("MOVE TOOLBAR UP\n");
	Set_ToolBar(0.8);

}
void MoveToolBarDown(){
	printf("MOVE TOOLBAR DOWN\n");
	Set_ToolBar(-0.6);

}
void MoveToolBarUp(unsigned char i){
	printf("MOVE TOOLBAR UP\n");
	Set_ToolBar(1);

}
void StopToolBar3(unsigned char i){
	Set_ToolBar(0);
}
void StopLoadGear(){
	printf("Stop LOAD GEAR\n");
	Set_GearIntake(0.1);
	MoveToolBarUp();
	startTimer(9,200,0, StopToolBar3);
	//MoveToolBarToGearDropoff();
}
void StopToolBar2(unsigned char i){
	Set_ToolBar(0);
	//startTimer(9,300,MoveToolBarUp, StopToolBar3);
}

void StartLoadGear(){

	printf("Start LOAD GEAR\n");
	selectCamera=1;
	cam1DriveMode=2;
	Set_GearIntake(0.3);
	MoveToolBarDown();
	startTimer(8,200,0,StopToolBar2);
	//MoveToolBarToGearPickup();
}
void MoveToolBarDownSlow(){
	printf("MOVE TOOLBAR DOWN SLOW\n");
	Set_ToolBar(-0.6);

}
void StopToolBar(){
	printf("STOP TOOLBAR\n");
	Set_ToolBar(0);
}
void StartDropGear(){
	printf("Start Drop GEAR\n");
	Set_GearIntake(-0.3);
	selectCamera=1;
	MoveToolBarDownSlow();
	//MoveToolBarToGearDropoff();
}
void StopDropGear(){
	printf("Stop Drop GEAR\n");
	Set_GearIntake(0.0);
	StopToolBar();
	//MoveToolBarToGearPickup();
}
void StartLoadBall(){
	printf("StartLOADBALL\n");
	Set_BallIntake(1.0);
}
void StopLoadBall(){
	printf("StopLOADBALL\n");
	Set_BallIntake(0);
}




void MoveTurretTo0(unsigned char){
	MoveTurretToPOS(0);
}
void MoveTurretTo90(unsigned char){
	MoveTurretToPOS(90);
}
void MoveTurretTo180(unsigned char){
	MoveTurretToPOS(180);
}

float curSeekLoc= 0;
float drive = 0;


float sampleTrack1 = 0;
float rawLoc1 = 0;
float lastRevNeg1  = 0;
float lastRevPos1 = 0;
float AnalogSample1 = 0;
float revCount1 = 0;
float encZero1 = 0;

void ReadEncoderSlow()
{
	float minLevel = 0;
	float delay=0;
	float newSample = robott->pot->Get()/2000;
	if (newSample < 0) newSample = 0;
	if (newSample > 5) newSample = 5;


	//if ( i==aAzimuth ) newSample=5-newSample; //|| i==aHood


	if (newSample == sampleTrack1) return;

	sampleTrack1 = newSample;

	rawLoc1 = newSample;

	float sampleDelta = AnalogSample1 - newSample;

	AnalogSample1 = newSample;

		if (sampleDelta > minLevel)  // check for underflow wraparound
		{
			double curTime = frc::GetClock();

			if (curTime - lastRevPos1 > delay)
			{
				lastRevNeg1 = 0;

				lastRevPos1 = curTime;

				revCount1++;

			}
		}

		if (sampleDelta < -minLevel)  // check for overflow  wraparound
		{
			float curTime = frc::GetClock();

			if (curTime - lastRevNeg1 > delay)
			{
				lastRevPos1 = 0;

				lastRevNeg1 = curTime;

				revCount1--;

			}
		}
	float tmp = ((float)revCount1 * 1 +(AnalogSample1)) - encZero1;

	// if (i==aAzimuth) tmp=tmp*-5;
	// if (i==aHood   ) tmp=(tmp-bumperIsDown)*1000.0 ;
	// if (i==aArm    ) tmp=tmp* armRes;

	currentHopperPOS = tmp;

}
float autoDriveEnabled = 1;
void GoalAim(){
	selectCamera=2;
	//MoveTurretToPOS(90);
	printf("Goal Aim\n");
	//SetShooterToRPM(3000);
	//printf("BoilerGoalX: %d\n", BoilerGoalX);
	if(BoilerTargetGot){
		float delta = BoilerGoalX - BoilerGoalX1;
		float ydelta = BoilerGoalY - BoilerGoalY1;
		curSeekLoc = delta*GoalAimGain;
		if(abs(delta)>4){
			if(fabs(curSeekLoc)>maxGoalAimSpeed){curSeekLoc=maxGoalAimSpeed*(curSeekLoc/(fabs(curSeekLoc)));}
			Set_Turret(-curSeekLoc);

		}
		else{
			Set_Turret(0);
		}

		drive = ydelta*BoilerErrorScale;
		if(autoDriveEnabled){
		if(abs(ydelta)>4){
			if(fabs(drive)>maxPegDriveSpeed){drive=maxPegDriveSpeed*(drive/(fabs(drive)));}
			joyOverride=1;
			joyYOverride=drive;
		}
		else{
			joyOverride=0;
			joyYOverride=0;
		}
		}

		printf("GoalAim: xDelta=%3.3f xSpeed=%1.3f yDelta=%3.3f ySpeed=%1.3f\n", delta, curSeekLoc, ydelta, drive);
	}
	else{
		printf("GoalAim: No Target\n");
		Set_Turret(0);
		joyOverride=0;

	}




	//startTimer(4, 100, 0, MoveTurretTo0);
	//startTimer(5, 200, 0, MoveTurretTo90);
	//startTimer(6, 300, 0, MoveTurretTo180);
}

void stopGoalAim(){
	curSeekLoc=0;
	joyOverride=0;
	drive=0;
	Set_Turret(0);

}
float pegcurSeekLoc = 0;
float pegcurSeekLocY = 0;
float pegcurSeekLocX  = 0;
extern float PegGoalAngle;
void PegAim(){
	selectCamera=1;
	cam1DriveMode=1;
	if(PegTargetGot){
			float delta = PegGoalX - PegGoalX1;
			float ydelta = PegGoalY - PegGoalY1;
			pegcurSeekLoc =  delta*pegGoalAimScale;
			pegcurSeekLocY = ydelta*pegGoalDriveScale;
			pegcurSeekLocX = PegGoalAngle*0.8;
			if(fabs(pegcurSeekLoc)>maxPegAimSpeed){pegcurSeekLoc=maxPegAimSpeed*(pegcurSeekLoc/(fabs(pegcurSeekLoc)));}
			if(fabs(pegcurSeekLocY)>maxPegDriveSpeed){pegcurSeekLocY=maxPegDriveSpeed*(pegcurSeekLocY/(fabs(pegcurSeekLocY)));}
			joyOverride = 0;
			joyXOverride = 0;
			joyYOverride = 0;
			if(fabs(delta)>1){
						joyOverride = 1;
						joyZOverride = pegcurSeekLoc;
			}
			if(fabs(ydelta)>1){

						joyOverride = 1;
						joyYOverride = pegcurSeekLocY;
						}



			if(fabs(ydelta)<=1 && fabs(delta)<=1){
				joyOverride = 0;
				joyXOverride = 0;
				joyYOverride = 0;
				joyZOverride = 0;
			}
			printf("PegAim: xDelta=%3.3f xSpeed=%1.3f yDelta=%3.3f ySpeed=%1.3f xDelta2=%3.3f\n", delta, pegcurSeekLoc, ydelta, pegcurSeekLocY, PegGoalAngle);
		}
		else{
			printf("PegAim: No Target\n");
			joyOverride = 0;

		}
}
void resetAimAndShoot(){
	lastTime2=0;
	selectCamera=2;
	curSeekLoc=0;
	Set_Turret(0);
}
void stopPegAim(){
	pegcurSeekLoc=0;
	joyOverride = 0;

}


float lastHopperPOS;
float lastHopperTime;

void ShootBalls(){
	printf("ShootBalls at RPM: %f\n", hopperShootRPM);
	SetHopperToRPM(hopperShootRPM);
}
void DontShootBalls(){
	printf("DontShoot\n");
	Set_Hopper(0);
}
void StopShootingBalls(){
	float lastSetPos = Get_HopperPOS();
	printf("StopShooting\n");
	Set_Hopper(0);
	Set_Shooter(0);
	stopGoalAim();

}


float lastHopPosTime;
float lastHopPOS;
float lastHopTime;
/*
void FeedBalls(){
	float curPos = currentHopperPOS;
	double curTime = frc::GetClock();

				if (curTime - lastHopPosTime > 1)
				{
					MoveHopperToPOS(curPos+20);
					lastHopPosTime = curTime;

				}
	hopperSpeed=(lastHopPOS-curPos)/(curTime-lastHopTime);
	lastHopTime=frc::GetClock();
	lastHopPOS = curPos;
	printf("curPOS: %f lastHopPosTime %f\n", curPos, lastHopPosTime);
}
*/


void ShootingBalls(){

	double  cur = Get_ShooterSpeed();
		if(abs(cur) > abs(ShooterShootRPM)-200 && abs(cur) < abs(ShooterShootRPM) +200){
			printf("UpToSpeed %f\n", cur);
			ShootBalls();
			//ShootBalls2();
		}
		else{
			printf("ShooterNotAtSpeed: %f\n", cur);
			SetShooterToRPM(ShooterShootRPM);
			DontShootBalls();
		}



}
void AtonShootBalls(unsigned char i){
		printf("ShootBalls at RPM: %f\n", aton_HopperRPM);
		SetHopperToRPM(aton_HopperRPM);

}
void AtonShootingBalls(){

	SetShooterToRPM(-aton_ShooterRPM);
	startTimer(11,80,0,AtonShootBalls);




}

/*
void ManualShooter(){
	if(Get_ShooterSetPoint > 0){
		printf("ManualShooter Off\n");
		SetShooterToRPM(0);
	}
	else{
		printf("ManualShooter On\n");
		SetShooterToRPM(3000);
	}
}
*/
void switchCam(){

	if(selectCamera==1){
		printf("switching camera2\n");
		selectCamera=2;
	}
	else if(selectCamera==2){
		printf("switching camera1\n");
			selectCamera=1;
		}
}
void switchDriveMode(){

	if(cam1DriveMode==1){
		printf("switching camera drive mode 2\n");
		cam1DriveMode=2;
	}
	else if(cam1DriveMode==2){
		printf("switching camera drive mode 1\n");
		cam1DriveMode=1;
		}
}


void UpdateVariables(){

		ToolBarLoc=Get_WheelLocation();;
		ToolBarDest=(robott->toolbarCAN->GetSetpoint());              
		ToolBarVolts=(robott->toolbarCAN->GetOutputVoltage());        
		ToolBarAmps=(robott->toolbarCAN->GetOutputCurrent());         

	    ShooterRPM1=(robott->shooterCAN->GetSpeed());                 
		ShooterDest=(robott->shooterCAN->GetSetpoint());              
		ShooterVolts=(robott->shooterCAN->GetOutputVoltage());        
		ShooterAmps=(robott->shooterCAN->GetOutputCurrent());         

		Winch1Speed=(robott->winch1CAN->Get());
		Winch1Volts=(robott->winch1CAN->GetOutputVoltage());
		Winch1Amps=(robott->winch1CAN->GetOutputCurrent());

		Winch2Speed=(robott->winch2CAN->Get());
		Winch2Volts=(robott->winch2CAN->GetOutputVoltage());
	    Winch2Amps=(robott->winch2CAN->GetOutputCurrent());

	    IntakeRightSpeed=(robott->gearRightRollerCAN->Get());             
	    IntakeRightVolts=(robott->gearRightRollerCAN->GetOutputVoltage());
	    IntakeRightAmps=(robott->gearRightRollerCAN->GetOutputCurrent()); 

	    IntakeLeftSpeed=(robott->gearLeftRollerCAN->Get());               
	    IntakeLeftVolts=(robott->gearLeftRollerCAN->GetOutputVoltage());  
	    IntakeLeftAmps=(robott->gearLeftRollerCAN->GetOutputCurrent());   

	//    BallIntakeSpeed=(robott->intakeRollerCAN->Get());
	 //   BallIntakeVolts=(robott->intakeRollerCAN->GetOutputVoltage());
	  //  BallIntakeAmps=(robott->intakeRollerCAN->GetOutputCurrent());

	    HopperSpeed=(robott->hopperCAN->Get());                           
	    HopperVolts=(robott->hopperCAN->GetOutputVoltage());              
	    HopperAmps=(robott->hopperCAN->GetOutputCurrent());               

	    HopperPos=robott->hopperCAN->GetPosition();
	    HopperRPM=(robott->hopperCAN->GetSpeed());

	    GyroX=(imu->GetAngleX());
	    GyroY=(imu->GetAngleY());
	    GyroZ=(imu->GetAngleZ());

		TurretPOS = Get_TurretPOS();

	}
float curTime;
float posCount;
float allHopperSpeed;
	void UpdateEncoders(){
			ReadEncoderSlow();
			//robott->hopperCAN->SetAnalogPosition(currentHopperPOS);
			/*
			curTime = frc::GetClock();
			allHopperSpeed += (currentHopperPOS-lastHopperPOS)/(curTime-lastHopperTime);
			posCount++;
			lastHopperPOS = currentHopperPOS;
			lastHopperTime = curTime;

			if(posCount>=10){
				hopperSpeed=allHopperSpeed/posCount;
				robott->hopperCAN->SetAnalogPosition(hopperSpeed);
				posCount = 0;
				allHopperSpeed = 0;
			}*/


	}

void setAntiBackDrive(float pos){
	robott->antibackdrive->SetAngle(pos);
}
void movedown(unsigned char i){
	MoveToolBarDown();
}
void dropPeg_3(unsigned char i){
	StopToolBar();

}

void dropPeg_2(unsigned char i){
	aton_move(-0.600,0,250);
	startTimer(8, 300, 0, dropPeg_3);
}
void dropPeg_1(unsigned char i){
	MoveToolBarDown();
	startTimer(8, 75, movedown, dropPeg_2);
}
void dropPeg(){
	Set_GearIntake(-0.4);
	MoveToolBarDown();
	startTimer(8, 20, 0, dropPeg_1);
}


void ShootBalls2(){
	printf("ShootBalls at Speed: %f\n", HopperShootSpeed);
	Set_Hopper(HopperShootSpeed);
}

void MoveAntiBackDriveToPOS(float pos){
	printf("MoveAntiBackDriveToPOS %f\n", pos);
	robott->antibackdrive->SetPosition(pos);
}
void EngageAntiBackDrive(){
	 MoveAntiBackDriveToPOS(-30);
}

void DegageAntiBackDrive(){
	 MoveAntiBackDriveToPOS(10);
}
void StopAntiBackDrive(){
	printf("StopAntiBAckDrive\n");
}


