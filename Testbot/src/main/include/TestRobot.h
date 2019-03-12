#pragma once

#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include "controllers/CurtinControllers.h"

#include "CurtinCtre.h"
#include "Gearbox.h"
#include "Drivetrain.h"

#include "DriveStrategies.h"


class Robot : public frc::TimedRobot, protected curtinfrc::StrategyController {
 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  curtinfrc::controllers::XboxController xbox{0};
  curtinfrc::controllers::SmartControllerGroup contGroup{xbox};

  curtinfrc::Talon *leftMotors[1], *rightMotors[1];
  curtinfrc::Gearbox *left, *right;
  curtinfrc::Drivetrain *drivetrain;
};
