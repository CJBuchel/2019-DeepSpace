// Simulation harness for the curtinfrc::TalonSrx.

#include "CurtinCtre.h"
#include "simulation/ctre_sim.h"

using namespace curtinfrc;

static std::map<int, simulation::ctre::talon_data> _talons;
static std::map<int, simulation::ctre::srx_data> _srxs;
static std::map<int, simulation::ctre::spx_data> _spxs;

std::map<int, simulation::ctre::talon_data> &simulation::ctre::all_talons() {
  return _talons;
}

std::map<int, simulation::ctre::srx_data> &simulation::ctre::all_srxs() {
  return _srxs;
}

std::map<int, simulation::ctre::spx_data> &simulation::ctre::all_spxs() {
  return _spxs;
}


// Talon

Talon::Talon(int port) : actuators::MotorVoltageController(this) {
  _talons[port] = simulation::ctre::talon_data{};
  _talons[port].port = port;
  _port = port;
}

Talon::~Talon() {
  _talons.erase(_port);
}

void Talon::SetUpdateRate(int hz) {
  // no op in sim
}

int Talon::GetPort() {
  return (int) _port;
}

void Talon::SetInverted(bool invert) {
  _talons[_port].inverted = true;
}

bool Talon::GetInverted() const {
  return _talons[_port].inverted;
}

void Talon::Disable() {
  _talons[_port].mode = ControlMode::Disabled;
}

void Talon::Set(double speed) {
  Set(ControlMode::PercentOutput, speed);
}

void Talon::Set(Talon::ControlMode mode, double value) {
  _talons[_port].mode = mode;
  _talons[_port].value = value;
  _value = value;
}

Talon::ControlMode Talon::GetMode() {
  return _talons[_port].mode;
}


// Talon SRX

TalonSrx::TalonSrx(int port, int encoderTicksPerRotation) : actuators::MotorVoltageController(this), Encoder::Encoder(encoderTicksPerRotation) {
  _srxs[port] = simulation::ctre::srx_data{};
  _srxs[port].port = port;
  _port = port;
}

TalonSrx::~TalonSrx() {
  _srxs.erase(_port);
}

void TalonSrx::SetUpdateRate(int hz) {
  // no op in sim
}

int TalonSrx::GetPort() {
  return (int) _port;
}

void TalonSrx::SetInverted(bool invert) {
  _srxs[_port].inverted = true;
}

bool TalonSrx::GetInverted() const {
  return _srxs[_port].inverted;
}

void TalonSrx::Disable() {
  _srxs[_port].mode = ControlMode::Disabled;
}

void TalonSrx::Set(double speed) {
  Set(ControlMode::PercentOutput, speed);
}

void TalonSrx::Set(TalonSrx::ControlMode mode, double value) {
  _srxs[_port].mode = mode;
  _srxs[_port].value = value;
  _value = value;
}

TalonSrx::ControlMode TalonSrx::GetMode() {
  return _srxs[_port].mode;
}

int TalonSrx::GetSensorPosition() {
  return _srxs[_port].sensor_pos;
}

int TalonSrx::GetSensorVelocity() {
  return _srxs[_port].sensor_vel;
}

void TalonSrx::ZeroEncoder() {
  _srxs[_port].sensor_pos = 0;
}

void TalonSrx::LoadConfig(TalonSrx::Configuration &config) {
  _srxs[_port].config = config;
}

TalonSrx::Configuration TalonSrx::SaveConfig() {
  return _srxs[_port].config;
}


// Victor SPX

VictorSpx::VictorSpx(int port) : actuators::MotorVoltageController(this) {
  _spxs[port] = simulation::ctre::spx_data{};
  _spxs[port].port = port;
  _port = port;
}

VictorSpx::~VictorSpx() {
  _spxs.erase(_port);
}

void VictorSpx::SetUpdateRate(int hz) {
  // no op in sim
}

int VictorSpx::GetPort() {
  return (int) _port;
}

void VictorSpx::SetInverted(bool invert) {
  _spxs[_port].inverted = true;
}

bool VictorSpx::GetInverted() const {
  return _spxs[_port].inverted;
}

void VictorSpx::Disable() {
  _spxs[_port].mode = ControlMode::Disabled;
}

void VictorSpx::Set(double speed) {
  Set(ControlMode::PercentOutput, speed);
}

void VictorSpx::Set(VictorSpx::ControlMode mode, double value) {
  _spxs[_port].mode = mode;
  _spxs[_port].value = value;
  _value = value;
}

VictorSpx::ControlMode VictorSpx::GetMode() {
  return _spxs[_port].mode;
}

void VictorSpx::LoadConfig(VictorSpx::Configuration &config) {
  _spxs[_port].config = config;
}

VictorSpx::Configuration VictorSpx::SaveConfig() {
  return _spxs[_port].config;
}
