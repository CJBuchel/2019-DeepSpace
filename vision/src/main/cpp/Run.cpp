#include "Run.h"
#include "TapeInit.h"
#include "Capture.h"
#include "TapeProcessing.h"
#include "Display.h"

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <iostream>

#include <cameraserver/CameraServer.h>
#include <networktables/NetworkTableInstance.h>
#include <cscore.h>

#include "devices/kinect.h"

using namespace cv;
using namespace std;


void curtin_frc_vision::run() {
  curtin_frc_vision::TapeInit();
  
  while(true) {
    curtin_frc_vision::Capture();
    curtin_frc_vision::TapeProcessing();
    curtin_frc_vision::Display();
  }
  
}