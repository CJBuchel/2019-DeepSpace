#include "Process.h"

#include <stdio.h>
#include <iostream>

#include <cameraserver/CameraServer.h>
#include <cscore.h>

#include "devices/kinect.h"

using namespace cv;
using namespace std;
//Set _capture as a Capture object reference
Process::Process(Capture &capture) : _capture(capture) {}

Capture &Process::GetCapture() {
  return _capture;
}

// Copiers
void Process::CopyImgOriginal(cv::Mat &imgOriginal) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgOriginal.copyTo(imgOriginal);
}

void Process::CopyProcessed(cv::Mat &imgProcessed) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgProcessed.copyTo(imgProcessed);
}

void Process::CopyImgBallThresh(cv::Mat &imgBallThresh) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgBallThresh.copyTo(imgBallThresh);
}

void Process::CopyImgBallTrack(cv::Mat &imgBallTrack) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgBallTrack.copyTo(imgBallTrack);
}

void Process::CopyImgHatchThresh(cv::Mat &imgHatchThresh) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgHatchThresh.copyTo(imgHatchThresh);
}

void Process::CopyImgHatchTrack(cv::Mat &imgHatchTrack) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgHatchTrack.copyTo(imgHatchTrack);
}

void Process::CopyImgTapeThresh(cv::Mat &imgTapeThresh) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgTapeThresh.copyTo(imgTapeThresh);
}

void Process::CopyImgTapeTrack(cv::Mat &imgTapeTrack) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _imgTapeTrack.copyTo(imgTapeTrack);
}

bool Process::GetValid() {
  return _imgTapeThresh.rows > 0;
}

std::string Process::GetProcessType() {
  return processType;
}

void Process::Init() {
  _videoMode = _capture.GetVideoMode();
	_imgOriginal = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
  _imgProcessed = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};

  _imgBallThresh = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
  _imgBallTrack = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};

  _imgHatchThresh = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
  _imgHatchTrack = cv::Mat{_videoMode.height, _videoMode.width, CV_8UC3};
}

void Process::Periodic() {}
