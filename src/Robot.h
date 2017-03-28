#ifndef ROBOT_H_
#define ROBOT_H_
#include <cmath>
#include <memory>
#include <Timer.h>
#include <Servo.h>
#include <CameraServer.h>
#include <RobotDrive.h>
#include <Joystick.h>
#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <AnalogGyro.h>

#include "OI.h"


class Robot: public frc::IterativeRobot {

public:
	static std::unique_ptr<OI> oi;
	//static std::shared_ptr<CameraSubsystem> camerasubsystem;

	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;



private:
	//Autonomous autonomousCommand;
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();




};

#endif  // ROBOT_H_
