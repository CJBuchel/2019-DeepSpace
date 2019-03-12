#include "DriveStrategies.h"

#include <cmath>

using namespace curtinfrc;
using namespace curtinfrc::controllers;

BaseDrivetrainTeleopStrategy::BaseDrivetrainTeleopStrategy(std::string name, Drivetrain &drivetrain, SmartControllerGroup &contGroup)
    : Strategy(name), _drivetrain(drivetrain), _contGroup(contGroup) {
  Requires(&drivetrain);
  SetCanBeInterrupted(true);
  SetCanBeReused(true);
} 

DrivetrainManualStrategy::DrivetrainManualStrategy(Drivetrain &drivetrain, SmartControllerGroup &contGroup) : BaseDrivetrainTeleopStrategy("Drivetrain Manual", drivetrain, contGroup) { }

void DrivetrainManualStrategy::OnUpdate(double dt) {
  double joyForward = 0, joyTurn = 0;
  
  joyForward = -_contGroup.Get(tAxis(0, 0)) * 0.9;
  joyForward *= std::abs(joyForward);

  joyTurn = _contGroup.Get(tAxis(0, 1)) * 0.7;
  // joyTurn *= abs(joyTurn);

  double leftSpeed = joyForward + joyTurn;
  double rightSpeed = joyForward - joyTurn;

  _drivetrain.Set(leftSpeed, rightSpeed);
}

DrivetrainAngleStrategy::DrivetrainAngleStrategy(Drivetrain &drivetrain, control::PIDGains gains, double angle) : Strategy("Drivetrain Angle"), _drivetrain(drivetrain), _pid(gains) {
  Requires(&drivetrain);
  SetCanBeInterrupted(true);
  SetCanBeReused(false);

  _angle = fmod(angle + (_drivetrain.GetInverted() ? 180 : 360), 360);
  _pid.SetSetpoint(_angle);
  _pid.SetWrap(360.0);

  _drivetrain.SetExternalLoop([&](Drivetrain &, double dt) {
    double offset = _pid.Calculate(_drivetrain.GetConfig().gyro->GetAngle(), dt);
    return std::pair<double, double>(offset, -offset);
  });
}

void DrivetrainAngleStrategy::OnUpdate(double dt) {
  if (_pid.IsDone())
    SetDone();
}
