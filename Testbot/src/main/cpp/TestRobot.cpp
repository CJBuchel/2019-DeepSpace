#include "TestRobot.h"

#include <actuators/VoltageController.h>

#include <math.h>
#include <iostream>

double lastTimestamp;

void Robot::RobotInit() {
  lastTimestamp = frc::Timer::GetFPGATimestamp();
  
  leftMotors[0] = new frc::Talon(1);
  leftMotors[0]->SetInverted(false);
  left = new curtinfrc::Gearbox{ new curtinfrc::actuators::MotorVoltageController(new frc::SpeedControllerGroup(*leftMotors[0])), nullptr};

  rightMotors[0] = new frc::Talon(0);
  rightMotors[0]->SetInverted(true);
  right = new curtinfrc::Gearbox{ new curtinfrc::actuators::MotorVoltageController(new frc::SpeedControllerGroup(*rightMotors[0])), nullptr};

  curtinfrc::DrivetrainConfig drivetrainConfig{ *left, *right, &gyro, 0.58, 0.48, 0.07, 20 };
  drivetrain = new curtinfrc::Drivetrain(drivetrainConfig);
  drivetrain->SetDefault(std::make_shared<DrivetrainManualStrategy>(*drivetrain, contGroup));
  drivetrain->GetConfig().gyro->Reset();
  // drivetrain->StartLoop(100);

  Register(drivetrain);
}

void Robot::RobotPeriodic() {
  double dt = frc::Timer::GetFPGATimestamp() - lastTimestamp;
  lastTimestamp = frc::Timer::GetFPGATimestamp();

  // drivetrain->Set(0.25, 0.25);

  drivetrain->Update(dt);
  // Update(dt);
}

void Robot::AutonomousInit() {
  double offset = drivetrain->GetConfig().gyro->GetAngle();
  offset += 0; // get angle from nt
  // Schedule(std::make_shared<DrivetrainAngleStrategy>(*drivetrain, curtinfrc::control::PIDGains("Drivetrain Align", 0.3, 0, 0.04), offset));
}
void Robot::AutonomousPeriodic() {
  std::cout << tapeAngleEntry.GetDouble(0) << std::endl;
}

void Robot::TeleopInit() {
  // Schedule(drivetrain->GetDefaultStrategy());
}
void Robot::TeleopPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}
