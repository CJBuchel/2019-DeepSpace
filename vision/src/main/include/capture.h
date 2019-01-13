#pragma once

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