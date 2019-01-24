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

extern std::mutex classMutexLocking;
extern std::condition_variable condVar;

class Process : public Runnable {
 public:
  virtual void Init() override;
  virtual void Periodic() override;

  Process(Capture &capture);
  Capture &GetCapture();
  void CopyImgTrack(cv::Mat &imgTrack);
  void CopyImgOriginal(cv::Mat &imgOriginal);
  std::mutex mutex();
  bool GetProcessReady();
  virtual bool GetDerivedReady() = 0;
  

 protected:
  std::mutex _classMutex;
  Capture &_capture;
  cs::VideoMode _videoMode;
  cv::Mat _imgTrack;
  cv::Mat _imgOriginal;
  bool _processReady = false;
  bool _derivedReady = false;
  
};