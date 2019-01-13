#pragma once

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

class Process {
 public:
  static Process *GetInstance();
  void Run();
  void Start();
  cv::Mat GetDrawing();
  cv::Mat GetGreenHueImage();
  cv::Mat GetOrangeHueImage();
};