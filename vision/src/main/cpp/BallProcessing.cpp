#include "Display.h"
#include "Capture.h"
#include "BallProcessing.h"

#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <iostream>
#include <condition_variable>
#include <string>
#include <thread>
#include <mutex>

#include <cameraserver/CameraServer.h>
#include <cscore.h>

#include "devices/kinect.h"

cv::RNG rng(12345);
cv::Rect bounding_rect;
int thresh = 100;
float height_offset;
float width_offset;
float width_goal = 320;
float height_goal = 240;

void BallProcessing::Init() {
  std::cout << "BallProcessing Init Started" << std::endl;
  Process::Init();
  std::cout << "BallProcessing Init Ended" << std::endl;
}

void BallProcessing::Periodic() {
  std::cout << "BallProcessing Periodic Started" << std::endl;
  

  {
    std::lock_guard<std::mutex> lk(_classMutex);
    _ready = true;
    std::cout << "main() signals data ready for processing\n";
  }
  _conVar.notify_all();
 
  // wait for the worker
  {
    std::unique_lock<std::mutex> lk(_classMutex);
    _conVar.wait(lk);
  }
 


  if (_capture.IsValidFrame()) {
    /* cv::Mat bgrThreshInput = _capture.CopyCaptureMat();
    double bgrThreshBlue[] = {0.0, 127.0};
    double bgrThreshGreen[] = {200.0, 255.0};		//thresholding values for finding green
    double bgrThreshRed[] = {0.0, 127.0}; */

    std::cout << "Origin Image Found" << std::endl;
    // Threshold the HSV image, keep only the green pixels (RetroBall)
    cv::cvtColor(_imgOriginal, _imgTrack, cv::COLOR_RGB2HSV);

    // Contours Blocks (Draws a convex shell over the thresholded image.)

    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> filteredContoursBall;
    std::vector<std::vector<cv::Point>> filteredHullsBall;
    std::vector<cv::Rect> ir_rects;
    int active_contour;
    cv::Scalar hsl_low, hsl_high;
    bool show_window;

    double largestArea = 0.0;
    active_contour = -1;
    // Filters size for Reflective Ball
    cv::inRange(_imgTrack, cv::Scalar(7, 100, 100), cv::Scalar(20, 255, 255), _imgTrack); // get img via getter
    cv::findContours(_imgTrack, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);

    for (int i = 0; i < contours.size(); i++) {
      std::vector<cv::Point> contour = contours[i];
      cv::Rect r = cv::boundingRect(contour);
      
      double area = cv::contourArea(contour);
      if (area > 300.0) {
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double solidity = 100 * area / contourArea(hull);

        if (solidity < 60.0) {
          if (area > largestArea) {
            largestArea = area;
            active_contour = filteredContoursBall.size();
          }
          filteredContoursBall.push_back(contour);
          filteredHullsBall.push_back(hull);
          ir_rects.push_back(r);
        }
      }
    }

    /// Detect edges using Canny
    cv::Canny(_imgTrack, _imgTrack, thresh, thresh * 2);

    /// Find contours
    std::vector<cv::Vec4i> hierarchy;

    /// Find the convex hull object for each contour
    std::vector<std::vector<cv::Point>> hullBall(filteredContoursBall.size());
    for (size_t i = 0; i < filteredContoursBall.size(); i++) {
      cv::convexHull(filteredContoursBall[i], hullBall[i]);
    }

    /// Draw filteredContours + hull results
    _imgTrack = cv::Mat::zeros(_imgTrack.size(), CV_8UC3);
    std::vector<cv::Rect> boundRectBall( filteredContoursBall.size() );

    for (size_t i = 0; i < filteredContoursBall.size(); i++) {
      cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
      cv::drawContours(_imgTrack, filteredContoursBall, (int)i, color);
      cv::drawContours(_imgTrack, hullBall, (int)i, color);
    }

    for (size_t i = 0; i < filteredContoursBall.size(); i++) {
      cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
      cv::drawContours(_imgTrack, filteredContoursBall, (int)i, color);
      cv::drawContours(_imgTrack, hullBall, (int)i, color);
    }

    /// Find contoursBox
    /// Approximate contoursBox to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point>> hullBall_poly(hullBall.size());
    std::vector<cv::Point2f> centerBall(hullBall.size());
    std::vector<float> radiusBall(hullBall.size());

    for(int i = 0; i < hullBall.size(); i++) {
      approxPolyDP(cv::Mat(hullBall[i]), hullBall_poly[i], 3, true);
      boundRectBall[i] = cv::boundingRect(cv::Mat(hullBall_poly[i]));
      cv::minEnclosingCircle((cv::Mat)hullBall_poly[i], centerBall[i], radiusBall[i]);
    }


    /// Draw polygonal contour + bonding rects + circles
    for(int i = 0; i < hullBall.size(); i++) {
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
      cv::drawContours(_imgTrack, hullBall_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
      bounding_rect = cv::boundingRect(filteredContoursBall[i]); // Find the bounding rectangle for biggest contour
      cv::rectangle(_imgTrack, boundRectBall[i].tl(), boundRectBall[i].br(), color, 2, 8, 0);
      cv::circle(_imgTrack, centerBall[i], (int)radiusBall[i], color, 2, 8, 0);
    }


    //_____________________Center Calcs______(Calculates the center from Border Box, And calculates X,Y Offset)_______ Ok.. it's suppose to calculate from borderbox, but not yet. using hull instead

    std::vector<cv::Moments> muBall(hullBall_poly.size()); // do we need this if we have mutex ? *
    for(int i = 0; i < hullBall_poly.size(); i++) {
      muBall[i] = moments(hullBall_poly[i], false);
    }

    // get the centroid of figures.
    std::vector<cv::Point2f> mcBall(hullBall_poly.size());
    for(int i = 0; i < hullBall_poly.size(); i++) {
      mcBall[i] = cv::Point2f(muBall[i].m10/muBall[i].m00 , muBall[i].m01/muBall[i].m00);
    }

    for(int i = 0; i < hullBall_poly.size(); i++) {
      cv::Scalar color = cv::Scalar(167,151,0); // B G R values
      cv::circle(_imgTrack, mcBall[i], 4, color, -1, 8, 0);

      // offsets from centerBall
      cv::Point centerBall = cv::Point((mcBall[i].x), (mcBall[i].y));
      width_offset = width_goal - centerBall.x;
      height_offset = height_goal - centerBall.y;
      std::cout << "Offset From CenterBall x,y =" << height_offset << "," << width_offset << std::endl;
    }
    std::cout << "BallProcessing Periodic Ended" << std::endl;
  }
}