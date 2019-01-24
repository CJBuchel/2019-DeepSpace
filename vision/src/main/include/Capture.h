#pragma once

#include <opencv2/core/core.hpp>
#include <cscore.h>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "Runnable.h"

class Capture : public Runnable {
 public:
  Capture(int port);
  int GetPort();
  void Init() override;
  void Periodic() override;

  cs::VideoMode GetVideoMode();
  void CopyCaptureMat(cv::Mat &captureMat);
  bool IsValidFrame();
  int GetCode();
  bool GetReady();

 private:
  std::mutex _classMutex;
  cs::UsbCamera _cam;
  cs::CvSink _sink{"USBSink"};
  cv::Mat _captureMat;
  cs::VideoMode _videoMode;
  bool _isValid = false;
  int camPort;
  int code;

  bool _captureReady = false;
  bool _processReady = false;
  
};