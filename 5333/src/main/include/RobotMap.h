#pragma once

#include <frc/SpeedControllerGroup.h>
#include <frc/Spark.h>
#include <frc/PowerDistributionPanel.h>

#include "CurtinCtre.h"
#include "controllers/CurtinControllers.h"
#include "Gearbox.h"
#include "actuators/BinaryServo.h"
#include "actuators/Compressor.h"
#include "actuators/DoubleSolenoid.h"
#include "actuators/VoltageController.h"
#include "sensors/Encoder.h"
#include "sensors/LimitSwitch.h"
#include "sensors/NavX.h"
#include "sensors/PressureSensor.h"

#include "control/PIDController.h"
#include "MotionProfiling.h"
// #include "strategy/MPStrategy.h"

#include "ControlMap.h"

#include "Drivetrain.h"
#include "Elevator.h"
#include "HatchIntake.h"
#include "BoxIntake.h"

struct RobotMap {
  curtinfrc::controllers::Joystick joy1{ 0 }; // Driver
  curtinfrc::controllers::Joystick joy2{ 1 }; // Co-Driver

  #if N_CONT == 2
  curtinfrc::controllers::SmartControllerGroup contGroup{ joy1, joy2 };
  #elif N_CONT == 3
  curtinfrc::controllers::XboxController xbox{ 2 };
  curtinfrc::controllers::SmartControllerGroup contGroup{ joy1, joy2, xbox };
  #endif

  struct DriveTrain {
    curtinfrc::Talon leftTalons{ 1 };
    curtinfrc::actuators::MotorVoltageController leftMotors = curtinfrc::actuators::MotorVoltageController::Group(leftTalons);
    curtinfrc::Gearbox leftGearbox{ &leftMotors, nullptr, 8.45 };

    curtinfrc::Talon rightTalons{ 0 };
    curtinfrc::actuators::MotorVoltageController rightMotors = curtinfrc::actuators::MotorVoltageController::Group(rightTalons);
    curtinfrc::Gearbox rightGearbox{ &rightMotors, nullptr, 8.45 };

    curtinfrc::sensors::NavX navx{frc::SPI::Port::kMXP, 200};
    curtinfrc::sensors::NavXGyro gyro{ navx.Angular(curtinfrc::sensors::AngularAxis::YAW) };

    curtinfrc::control::PIDGains gainsFOC{ "Drivetrain FOC", 0.008, 0, 0 };
    curtinfrc::control::PIDGains gainsAlign{ "Drivetrain Align", 0.3, 0, 0.04 };
    curtinfrc::PathfinderGains gainsPathfinder{ "Drivetrain Pathfinder", 24.0, 0, 1.5, 0.36, 0.08, 12.0 / 90.0 };    // PIDVAG

    curtinfrc::DrivetrainConfig config{ leftGearbox, rightGearbox, &gyro, 0.71, 0.71, 0.0762, 50 };
    curtinfrc::control::PIDGains gainsVelocity{ "Drivetrain Velocity", 1 };
  };

  DriveTrain drivetrain;


  struct ControlSystem {
    // vision
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

  ControlSystem controlSystem;
};
