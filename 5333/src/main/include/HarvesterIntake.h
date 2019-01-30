#pragma once

#include "intakes/DeployableIntake.h"
#include "StateDevice.h"
#include "Gearbox.h"
#include "CurtinControllers.h"

#include <frc/DoubleSolenoid.h>

using HarvesterIntakeState = curtinfrc::intakes::DeployableIntakeState;

struct HarvesterIntakeConfig : public curtinfrc::intakes::DeployableIntakeConfig {
  curtinfrc::Gearbox &motors;

  HarvesterIntakeConfig(curtinfrc::Gearbox &motorsIn, curtinfrc::actuators::BinaryActuator &actuatorIn) : curtinfrc::intakes::DeployableIntakeConfig(actuatorIn), motors(motorsIn) {};
};

class HarvesterIntake : public curtinfrc::intakes::DeployableIntake {
 public:
  HarvesterIntake(HarvesterIntakeConfig config) : DeployableIntake(config), _config(config) {};

 protected:
  virtual void DeployedPeriodic(HarvesterIntakeState state) override;
  virtual void DeployingPeriodic() override;
  virtual void StowingPeriodic() override;
  virtual void StowedPeriodic() override;

 private:
  HarvesterIntakeConfig _config;
};

class HarvesterIntakeController {
 public: 
  HarvesterIntakeController(HarvesterIntake &harvesterIntake, curtinfrc::Joystick &joy) : _harvesterIntake(harvesterIntake), _joy(joy) {};
  void Update(double dt);

  private:
  HarvesterIntake &_harvesterIntake;
  curtinfrc::Joystick &_joy;
};


