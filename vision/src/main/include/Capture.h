#pragma once

#include <opencv2/core/core.hpp>
#include <cscore.h>
#include <mutex>
#include <thread>

#include "Runnable.h"

class Capture : public Runnable {
 public:
  Capture(int port);
  int GetPort();
  void Init() override;
  void Periodic() override;

  cs::VideoMode GetVideoMode();
  void CopyCaptureMat(cv::Mat &captureMat);
  void Capture::CopyImgTrack(cv::Mat &imgTrack);
  void Capture::CopyImgOriginal(cv::Mat &imgOriginal);
  bool IsValidFrame();
  int GetCode();

 private:
  std::mutex _classMutex;
  cs::UsbCamera _cam;
  cs::CvSink _sink{"USBSink"};
  cv::Mat _captureMat;
  cv::Mat _imgTrack;
  cv::Mat _imgOriginal;
  cs::VideoMode _videoMode;
  bool _isValid = false;
  int camPort;
  int code;
};