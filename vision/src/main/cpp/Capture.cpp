#include "Capture.h"
#include "Process.h"

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <iostream>

#include "networktables/NetworkTableInstance.h"

#include <cameraserver/CameraServer.h>
#include <cscore.h>

#include "devices/kinect.h"

#include <mutex>
#include <condition_variable>

#include <stdlib.h>

using namespace cv;
using namespace std;

Capture::Capture(int port, int exposure) : _cam("USBCam", port) {_cam.SetExposureManual(exposure);}

// Getters
cs::VideoMode Capture::GetVideoMode() {
  if (_videoMode.height == 0 || _videoMode.width == 0) {
    std::unique_lock<std::mutex> lock(_classMutex);
    _initCondVar.wait(lock);
  }
  return _videoMode;
}

// Copiers
void Capture::CopyCaptureMat(cv::Mat &captureMat) {
  std::lock_guard<std::mutex> lock(_classMutex);
  _captureMat.copyTo(captureMat);
}

bool Capture::IsValidFrameThresh() {
  return _isValidThresh;
}

bool Capture::IsValidFrameTrack() {
  return _isValidTrack;
}

void Capture::SetCapturePort(int port) {
  _cam = cs::UsbCamera{"USBCam",port};
}

void Capture::Init() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto visionTable = inst.GetTable("VisionTracking");
  auto table = visionTable->GetSubTable("HatchTracking");
  HatchLeftSideEntry = table->GetEntry("Left Side"); 

  _sink.SetSource(_cam);
  _cam.SetResolution(640, 480);

  for (auto it : _cam.EnumerateProperties()) {
    std::cout << "Property: " << it.GetName() << " -> " << it.Get() << std::endl;
  }
  _videoMode = _cam.GetVideoMode();
  std::cout << "Width: " << _videoMode.width << " Height: " << _videoMode.height << std::endl;
  _captureMat = cv::Mat::zeros(_videoMode.height, _videoMode.width, CV_8UC3);
}

void Capture::Periodic() {
  _isValidThresh = _isValidTrack = _sink.GrabFrame(_captureMat) != 0;
}