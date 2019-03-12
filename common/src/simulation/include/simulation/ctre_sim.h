#pragma once

#include "CurtinCtre.h"

#include <map>

namespace simulation {

  class ctre {
   public:
    struct talon_data {
      int port;

      bool inverted;
      double value;
      curtinfrc::Talon::ControlMode mode = curtinfrc::Talon::ControlMode::Disabled;

      int sensor_pos;
      int sensor_vel;
    };

    struct srx_data {
      int port;

      bool inverted;
      double value;
      curtinfrc::TalonSrx::ControlMode mode = curtinfrc::TalonSrx::ControlMode::Disabled;

      int sensor_pos;
      int sensor_vel;

      curtinfrc::TalonSrx::Configuration config;
    };

    struct spx_data {
      int port;

      bool inverted;
      double value;
      curtinfrc::VictorSpx::ControlMode mode = curtinfrc::VictorSpx::ControlMode::Disabled;

      curtinfrc::VictorSpx::Configuration config;
    };

    static std::map<int, simulation::ctre::talon_data> &all_talons();
    static std::map<int, simulation::ctre::srx_data> &all_srxs();
    static std::map<int, simulation::ctre::spx_data> &all_spxs();
  };

}