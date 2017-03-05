#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <Servo.h>
#include <WPILib.h>
#include <Talon.h>
#include <cmath>
#include <ADXRS450_Gyro.h>
#include <iostream>
#include <Encoder.h>
#include <VictorSP.h>

class Robot: public frc::IterativeRobot {
public:
	ADXRS450_Gyro* gyro;
    frc::DigitalInput* limitSwitch;
    frc::DigitalOutput* indicatorLights;
    Encoder *encoder, *encoder2;

	void RobotInit() {
			CameraServer::GetInstance()->StartAutomaticCapture();
			 limitSwitch = new frc::DigitalInput(4);
			 indicatorLights = new frc::DigitalOutput(8);

		 	//frc::Wait(kUpdatePeriod);  Wait 5ms for the next update.
            gyro = new ADXRS450_Gyro(frc::SPI::Port::kOnboardCS0);
            gyro->Calibrate();
            encoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
            encoder2 = new Encoder(2, 3, false, Encoder::EncodingType::k4X);
            indicatorLights->DisablePWM();
            	}


private:
	frc::RobotDrive myRobot { 1, 2, 3, 4 };  // Robot drive system
	frc::Joystick stick { 0 };         // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::Timer timer;
	frc::Servo* cameraServo = new Servo(8);
    frc::Joystick m_stick { 0 };
    //frc::RobotDrive    { 1, 2, 3, 4 };
    frc::Servo* gateServoLeft =  new Servo(9);
    frc::Servo* gateServoRight =  new Servo(0);
    frc::Servo* gearServo =  new Servo(5);
    frc::Talon winch_motor { 6 };
    frc::Talon m2_motor  { 7 };
    frc::Talon ball_Shooter { 7 };
    frc::AnalogInput ultrasonic { 0 };
    static constexpr double kAngleSetpoint = 0.0;
    static constexpr double kVoltsPerDegreePerSecond = 0.0128;
    static constexpr int kGyroPort = 0;
	static constexpr double kUpdatePeriod = 0.005;
	static constexpr double kP = 0.005;
	double cameraDirection;
	double precisionSpeed;
	int count;
	double distanceTraveled, distanceTraveled2;
	bool autoComplete;
	bool HaveTurned;

	void AutonomousInit() override {
		autoComplete = false;
		timer.Reset();
		gyro->Calibrate();
		timer.Start();
		HaveTurned = false;
		encoder->SetDistancePerPulse(1);
		encoder2->SetDistancePerPulse(1);
		encoder->Reset();
		encoder2->Reset();
		enableLights(true);
		ResetServos();

	}

	void AutonomousPeriodic() override {
		// Drive for 2 seconds
			if(! autoComplete){
			Drive(100, 0.40, false);
			Drive(15, 0.25, true);
			//Got limit switch
			//no-do nothing
			//yes-open servos, wait
			//kick gearServo
			//reverse
			//close arms and gearServo
			if (IsSwitchPress()){
				OpenServo();
				KickGear();
				frc::Wait(0.4);
				Drive(-36, -0.30, false);
				ResetServos();
				Turn(-90);
				Drive(120, 0.40, false);
				Turn(90);
				Drive(120, 0.40, false);
				Turn(180);


			}

			autoComplete = true;
		}

	}
	void ResetServos(){
		gateServoLeft->SetAngle(160);
		gateServoRight->SetAngle(0);
		gearServo->SetAngle(10);
	}
	void KickGear(){
		gearServo->SetAngle(90);
	}
	void OpenServo(){
		gateServoLeft->SetAngle(85);
	   	gateServoRight->SetAngle(90);
	}
	void Log() {
		/*
		std::cout << "\nEncoder 1: " << encoder->GetRate() << "\n";
		std::cout << "Encoder 2: " << encoder2->GetRate() << "\n";
		std::cout << "Encoder 1 Distance: " << encoder->GetDistance() << "\n";
		std::cout << "Encoder 2 Distance: " << encoder2->GetDistance() << "\n";
		*/


	}

	double Straighten() {
		double a, b, value;
		a = encoder->GetDistance();
		b = encoder2->GetDistance();
		//value = (a+b)/-200; //competition robot
		value = (a+b)/200; //Practice robot

		return value;

		//TODO Should we reset to deal with overflow?
	}

	void TeleopInit() override {
		encoder->SetDistancePerPulse(1);
		encoder2->SetDistancePerPulse(1);
		cameraDirection = 1;
		gyro->Calibrate();
        precisionSpeed = 1;
        myRobot.ArcadeDrive(stick);
        myRobot.SetExpiration(0.1);
        count = 0;

	}


	void TeleopPeriodic() override {
;
		// Drive with arcade style (use right stick)

		//std::cout << gyro->GetAngle() << "|";
        double rate, rate2;
		rate = encoder->GetRate();
		rate2 = encoder2->GetRate();
		distanceTraveled = encoder->GetDistance();
		distanceTraveled2 = encoder2->GetDistance();
		//std::cout << " distance Traveled " << distanceTraveled << " " << distanceTraveled2 << std::endl;
		//std::cout << rate+rate2 << std::endl;


		/*******************************************************************************************
		 * Ouput ultrasonic reading every 75 passes
		 */
		if (count++ == 75){
			std::cout << "sonar " << ultrasonic.GetAverageVoltage() / 0.31 << std::endl;
			std::cout << "encoder rate " << rate  << "|"  <<  rate2 << std::endl;
			count = 0;
		}


//Flip motors with camera direction
           if(cameraDirection == 1){
        	   //myRobot.ArcadeDrive(-stick.GetY()*precisionSpeed,-stick.GetX()*precisionSpeed);// competition bot
        	   myRobot.ArcadeDrive(stick.GetY()*precisionSpeed,-stick.GetX()*precisionSpeed);// practice bot

        	   //frc::Wait(.004);

           }

           else if(cameraDirection != 1){
        	   //myRobot.ArcadeDrive(stick.GetY()*precisionSpeed,-stick.GetX()*precisionSpeed);// competition bot
        	   myRobot.ArcadeDrive(-stick.GetY()*precisionSpeed,-stick.GetX()*precisionSpeed); //practice bot
        	  // frc::Wait(.004);
           }


//Camera Servo
           if(m_stick.GetRawButton(5))
			{
				cameraServo->SetAngle(0); //regular driving
				cameraDirection = 1;

			}
			else if(m_stick.GetRawButton(6))
			{
				cameraDirection = 2;
				cameraServo->SetAngle(180);     //reverse driving

			}
			if(m_stick.GetRawButton(2))
			{
				ball_Shooter.Set(1);
			}
			else
			{
				ball_Shooter.Set(0);
			}

/**********************************************************************
 * Tune the speed of the motors with the throttle
 */
				precisionSpeed = (m_stick.GetAxis(frc::Joystick::kThrottleAxis)+1.8)/2;


			m_stick.GetAxis(frc::Joystick::kThrottleAxis);
					//std::cout << m_stick.GetAxis(frc::Joystick::kThrottleAxis) << std::endl;


// Gate Servo

			if(m_stick.GetRawButton(4))
			{
			    gateServoLeft->SetAngle(160);
			    gateServoRight->SetAngle(0);


			}
			else if(m_stick.GetRawButton(3))
            {
            	gateServoLeft->SetAngle(85);
            	gateServoRight->SetAngle(90);
            	gearServo->SetAngle(10);
            }


//Gear pushing Servo
			if(m_stick.GetRawButton(10))
			{
				gearServo->SetAngle(90);

			}



//Winch Motor
			if(m_stick.GetRawButton(12)){
				winch_motor.Set(-1);
			}

			else if(m_stick.GetRawButton(11)){
				winch_motor.Set(1);
			}
			else{
				winch_motor.Set(0);
			}

//Spring Spear Limit Switch
		checkSpear();


	}
	void TestPeriodic() override {
		lw->Run();
	}
	double Feather( double TotalDist, double DistTraveled){
		double returnValue = 0;

		if (TotalDist == 0)
				return 0;

		returnValue = (TotalDist - DistTraveled)/TotalDist;

		if (returnValue >= 1 )
			return 1;
		else if(returnValue <= -1)
			return -1;
		else if(returnValue < .5 && returnValue > .01)
			return 0.5;
		else if(returnValue > -.5 && returnValue < -.01)
			return -0.5;
		else
			return returnValue;


	}


	void Turn(float angle)
	{
		gyro->Reset();
		float targetHeading = gyro->GetAngle() + angle;

		float remainingTurn = targetHeading - gyro->GetAngle();

		while  (fabs(remainingTurn) > 0.05)  // we are within .5 degrees
		{
		//TODO Flip negative and positive for competition
			myRobot.ArcadeDrive(0.0, (remainingTurn < 0.0)? 0.45: -0.45);

			frc::Wait(.02);
			remainingTurn = targetHeading - gyro->GetAngle();
			std::cout << remainingTurn << std::endl;

		}
		Stop();
	}

	void Drive( double TotalDist, double speed, bool spearArmed) {
		double featherValue;
		do {
			featherValue = Feather(TotalDist, RecalcDistance());
			double s = speed * featherValue;
			if (s >= 0 ){
				myRobot.Drive(s, 1*Straighten());
			}
			else{
				myRobot.Drive(s, -1*Straighten());

			}
		}

		while (featherValue > 0.02 && !(spearArmed && IsSwitchPress()));
		if (IsSwitchPress()) {
			flashLights();
		}
		Stop();
	}
	double RecalcDistance() {
		//double Dist;
		double DistanceIN;
		//Dist = encoder->GetDistance();
		DistanceIN = encoder->GetDistance() /6.325;
		if (count++ == 25) {
			Log();
			count = 0;
		}
		return DistanceIN;


	}

	void Stop(){
		myRobot.Drive(0.0, 0.0);
		frc::Wait(1);
		encoder->Reset();
		encoder2->Reset();
	}

	/*void () {
		// Drive for 2 seconds
		if (timer.Get() < 5.0) {
			myRobot.Drive(0.25, Straighten()); // Drive forwards half speed
		} else if (timer.Get() < 6.0) {
			myRobot.Drive(0.0, 0.0);
		} else if (timer.Get() < 11.0) {
			myRobot.Drive(-0.25, Straighten() * -1);
		} else {
			myRobot.Drive(0.0, 0.0); // Stop robot
		}

		if (count++ == 25) {
			Log();
			count = 0;
		}
		frc::Wait(0.04);
	}*/


	/*void ExtraCrispy() {
		if (timer.Get() < 6.0) {
			myRobot.TankDrive(.15, ); // Drive forwards half speed
		} else if (timer.Get() < 7.0) {
			myRobot.TankDrive(0.0, 0.0);
		} else if (timer.Get() < 13.0) {
			myRobot.TankDrive(-0.15, );
		} else {
			myRobot.TankDrive(0.0, 0.0); // Stop robot
		}

		if (count++ == 25) {
			Log();
			count = 0;
		}
	}*/



	bool IsSwitchPress(){
		return !limitSwitch->Get();
		//Switch is backwards on proto
	}

	void flashLights(){
		for(int i = 0; i < 5; i++){
			enableLights(true);
			frc::Wait(0.1);
			enableLights(false);
			frc::Wait(0.1);
		}
	}

	void enableLights(bool onOff){
		indicatorLights->Set(onOff);
	}
	bool checkSpear() {
		//Spring Spear Limit Switch
		if (IsSwitchPress()) {
			enableLights(true);
			return true;
		} else {
			enableLights(false);
			return false;
		}
	}
};

START_ROBOT_CLASS(Robot)

