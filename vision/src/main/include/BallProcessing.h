#pragma once

#include "Process.h"

class BallProcessing : public Process {
 public:
  BallProcessing(Capture &capture) : Process(capture) {}

  void Init() override;
  void Periodic() override;
  bool GetBallProcessingReady();
  virtual bool GetDerivedReady() override;

 protected:
  Process &_process;
  bool _BallProcessingReady = false;
};