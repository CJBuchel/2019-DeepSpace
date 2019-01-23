#pragma once

#include "Capture.h"
#include "Runnable.h"

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <condition_variable>
#include <mutex>

class Process : public Runnable {
 public:
  virtual void Init() override;
  virtual void Periodic() override;

  Process(Capture &capture);
  Capture &GetCapture();
 protected:
  std::mutex _classMutex;
  std::condition_variable _conVar;
  bool _ready = false;
  bool _processed = false;

  Capture &_capture;
  cs::VideoMode _videoMode;
  cv::Mat _imgTrack;
  cv::Mat _imgOriginal;
};