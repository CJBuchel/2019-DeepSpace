#pragma once

#include <frc/Timer.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include "controllers/CurtinControllers.h"

#include "CurtinCtre.h"
#include "Gearbox.h"
#include "Drivetrain.h"

#include "sensors/NavX.h"

#include "DriveStrategies.h"


class Robot : public frc::TimedRobot, protected curtinfrc::StrategyController {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  curtinfrc::controllers::XboxController xbox{0};
  curtinfrc::controllers::SmartControllerGroup contGroup{xbox};

  frc::Talon *leftMotors[1], *rightMotors[1];
  curtinfrc::Gearbox *left, *right;

  curtinfrc::sensors::NavX navx{ frc::SPI::Port::kMXP, 200 };
  curtinfrc::sensors::NavXGyro gyro{ navx.Angular(curtinfrc::sensors::AngularAxis::YAW) };

  curtinfrc::Drivetrain *drivetrain;


  std::shared_ptr<nt::NetworkTable> visionTable = nt::NetworkTableInstance::GetDefault().GetTable("VisionTracking");
  std::shared_ptr<nt::NetworkTable> hatchTable = visionTable->GetSubTable("HatchTracking");
  std::shared_ptr<nt::NetworkTable> tapeTable = visionTable->GetSubTable("TapeTracking");
  
  nt::NetworkTableEntry hatchDistanceEntry  = hatchTable->GetEntry("Hatch Distance"),
                        hatchXoffsetEntry   = hatchTable->GetEntry("Hatch X Offset"),
                        hatchYoffsetEntry   = hatchTable->GetEntry("Hatch Y Offset"),
                        tapeDistanceEntry   = tapeTable->GetEntry("Distance"),
                        tapeAngleEntry      = tapeTable->GetEntry("Angle"),
                        tapeTargetEntry     = tapeTable->GetEntry("Target");
};
