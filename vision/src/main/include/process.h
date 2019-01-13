#pragma once

class Process {
 public:
  static Process *GetInstance();
  void Run();
  void Start();
  cv::Mat GetDrawing();
  cv::Mat GetGreenHueImage();
  cv::Mat GetOrangeHueImage();
};