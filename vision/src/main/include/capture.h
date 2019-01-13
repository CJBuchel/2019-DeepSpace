#pragma once

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

class Capture {
 public:
  static Capture *GetInstance();
  void Init();
  void Run();
  void Start(); // depends if threading is done here or in vision.cpp
  auto GetVideoModeTape();
  auto GetVideoModeBall();
  cv::Mat GetImgOriginalTape();
  cv::Mat GetImgOriginalBall();
  cv::Mat GetImgHSVTape();
  cv::Mat GetImgHSVBall();
};