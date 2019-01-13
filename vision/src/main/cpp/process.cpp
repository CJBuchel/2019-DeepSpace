#include "capture.h"
#include "process.h"

#include <cameraserver/CameraServer.h>
#include <cscore.h>
#include <thread>

using namespace cv;
using namespace std;

RNG rng(12345);
Rect bounding_rect;
int thresh = 100;
float height_offset;
float width_offset;
float width_goal = 320;
float height_goal = 240;

// Target vectors
vector<cv::Point2f> centres;
vector<bool> lefts;
vector<bool> rights;
vector<cv::Point2f> targets;
vector<float> angles;
vector<float> heights;
vector<float> distances;

cv::Mat drawing;
cv::Mat greenHueImage;
cv::Mat orangeHueImage;

static Process *process;

static Process *GetInstance() { // will this work ? are there multiple instances ?
  if (process == NULL) {
    process = new Process();
  }
  return process;
}

// Getters
cv::Mat GetDrawing() { return drawing; };
cv::Mat GetGreenHueImage() { return greenHueImage; };
cv::Mat GetOrangeHueImage() { return orangeHueImage; };

// these need to be passed to display
// drawing, greenHueImage (potentially orangeHueImage ?)

// these need to be passed to display and also pulled from capture
// imgOriginalTape, imgHSVTape, imgOriginalBall, imgHSVBall

void Run() {
  //Green Hue Processing Block
  //========================================================================================================
  //--------------------------------------------------------------------------------------------------------
  //========================================================================================================

  // Threshold the HSV image, keep only the green pixels (RetroTape)
  cv::inRange(Capture::GetInstance()->GetImgHSVTape(), cv::Scalar(35, 100, 30), cv::Scalar(78, 255, 255), greenHueImage); // get img via getter
  cv::inRange(Capture::GetInstance()->GetImgHSVBall(), cv::Scalar(10, 100, 100), cv::Scalar(20, 255, 255), orangeHueImage);

  //========================================================================================================
  //--------------------------------------------------------------------------------------------------------
  //========================================================================================================




  // Contours Blocks (Draws a convex shell over the thresholded image.)
  //________________________________________________________________________________________________________
  //________________________________________________________________________________________________________


  //KinectProcessor();
  //void processRGB(Mat mat);
  //void processIR(Mat mat);
      
  //kinectCondition t_condition;
  vector<vector<Point> > contours;
  vector<vector<Point> > filteredContoursTape;
  vector<vector<Point> > filteredContoursBall;
  vector<vector<Point> > filteredHullsTape;
  vector<vector<Point> > filteredHullsBall;
  vector<Rect> ir_rects;
  int active_contour;
  Scalar hsl_low, hsl_high;
  bool show_window;

  //cv::Mat tmp(image);

  // Usually this is used to convert it to HLS, but since it's an IR (monochrome) image,
  // there's no need to waste the CPU Time doing this 
  // cvtColor(tmp, tmp, CV_RGB2HLS);

  double largestArea = 0.0;
  active_contour = -1;
  // Filters size for Reflective Tape
  findContours(greenHueImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
  for (int i = 0; i < contours.size(); i++) {
    vector<Point> contour = contours[i];
    Rect r = boundingRect(contour);
    
    double area = contourArea(contour);
    if (area > 300.0) {
      vector<Point> hull;
      convexHull(contour, hull);
      double solidity = 100 * area / contourArea(hull);

      if (solidity < 60.0) {
        if (area > largestArea) {
          largestArea = area;
          active_contour = filteredContoursTape.size();
        }
        filteredContoursTape.push_back(contour);
        filteredHullsTape.push_back(hull);
        ir_rects.push_back(r);
      }
    }
  }
  // Filters size for Ball
  findContours(orangeHueImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
  for (int i = 0; i < contours.size(); i++) {
    vector<Point> contour = contours[i];
    Rect r = boundingRect(contour);
    
    double area = contourArea(contour);
    if (area > 300.0) {
      vector<Point> hull;
      convexHull(contour, hull);
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
  cv::Mat cannyTape_output;
  cv::Mat cannyBall_output;
  Canny(greenHueImage, cannyTape_output, thresh, thresh * 2);
  Canny(orangeHueImage, cannyBall_output, thresh, thresh * 2);

  /// Find contours
  //vector<vector<Point> > contours;
  cv::Mat thresholdTape_output;
  cv::Mat thresholdBall_output;
  vector<Vec4i> hierarchy;
  threshold( greenHueImage, thresholdTape_output, thresh, 255, THRESH_BINARY );
  threshold( orangeHueImage, thresholdBall_output, thresh, 255, THRESH_BINARY );
  //findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
  //findContours(greenHueImage, contours, hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  // ----Tape
  /// Find the convex hull object for each contour
  vector<vector<Point> >hullTape(filteredContoursTape.size());
  for (size_t i = 0; i < filteredContoursTape.size(); i++) {
    convexHull(filteredContoursTape[i], hullTape[i]);
  }

  /// Draw filteredContours + hull results
  drawing = Mat::zeros(cannyTape_output.size(), CV_8UC3);
  vector<Rect> boundRectTape( filteredContoursTape.size() );


  for (size_t i = 0; i < filteredContoursTape.size(); i++) {
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    drawContours(drawing, filteredContoursTape, (int)i, color);
    drawContours(drawing, hullTape, (int)i, color);
  }

  // -------Ball

  /// Find the convex hull object for each contour
  vector<vector<Point> >hullBall(filteredContoursBall.size());
  for (size_t i = 0; i < filteredContoursBall.size(); i++) {
    convexHull(filteredContoursBall[i], hullBall[i]);
  }

  /// Draw filteredContours + hull results
  drawing = Mat::zeros(cannyBall_output.size(), CV_8UC3);
  vector<Rect> boundRectBall( filteredContoursBall.size() );



  for (size_t i = 0; i < filteredContoursTape.size(); i++) {
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    drawContours(drawing, filteredContoursTape, (int)i, color);
    drawContours(drawing, hullTape, (int)i, color);
  }

  for (size_t i = 0; i < filteredContoursBall.size(); i++) {
    Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    drawContours(drawing, filteredContoursBall, (int)i, color);
    drawContours(drawing, hullBall, (int)i, color);
  }

  //________________________________________________________________________________________________________
  //________________________________________________________________________________________________________


  //Get RotatedRectangles 
  centres.clear(); //clear the vectors
  heights.clear();
  lefts.clear();
  rights.clear();

  for (int i=0; i<filteredContoursTape.size(); i++) {
    
    cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

    float angle = rotatedRect.angle;

    cv::Point2f centre = rotatedRect.center;

    cv::Point2f rectPoints[4];
    rotatedRect.points(rectPoints);

    float min = rectPoints[0].y;
    float max = rectPoints[0].y;
    
    for (int j=1; j<4; j++) { //find the minimum and maximum y-values of each rectangle
      if (rectPoints[j].y > max) {
        max = rectPoints[j].y;
      }
      if (rectPoints[j].y < min) {
        min = rectPoints[j].y;
      }
    }

    float height = max - min; //get the height of each rectangle

    centres.push_back(centre);
    heights.push_back(height);

    if (angle > 10 && angle < 19) {
      rights.push_back(true);
      lefts.push_back(false);
    }
    else if (angle < -10 && angle > -19) {
      rights.push_back(false);
      lefts.push_back(true);
    }
    else {
      rights.push_back(false);
      lefts.push_back(false);
    }

    std::stringstream ss;	ss<<angle; //magic shit, idk
    std::stringstream hei;	hei<<height;	
    cv::putText(drawing, ss.str() + " height:" + hei.str(), centre + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255)); //label the angle on each rectangle
  }

  int leftmost = -1;
  float leftPos = 1280;
  targets.clear();
  angles.clear();
  distances.clear();

  for (int i=0; i<filteredContoursTape.size(); i++) {
    if (lefts[i]) { //checks if current iteration is a left
      for (int j=0; j<filteredContoursTape.size(); j++) {
        if (rights[j] && centres[j].x < leftPos && centres[j].x > centres[i].x) { //checks if nested iteration is a right and left of the last checked one
          leftmost = j;
        }
      }

      if (leftmost > -1) {
        targets.push_back((centres[i]+centres[leftmost])/2); //adds the Points2f position of each target to a vector
        distances.push_back(200/(heights[i]+heights[leftmost])); //adds the estimated distance to each target. Calibrate by changing the number.
      }
    }
  }

  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

  for (int i=0; i<targets.size(); i++) {
    std::stringstream dis;	dis<<distances[i];
    cv::rectangle(drawing, targets[i] + Point2f(-3,-3), targets[i] + Point2f(3,3), color, 2); //draw small rectangle on target locations
    cv::putText(drawing, dis.str(), targets[i] + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255));
  }





  // Bounding Box Block, (Draws a border around the processed image)
  //,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

  //vector<vector<Point> > contoursBox;

  /// Detect edges using Threshold
  threshold( greenHueImage, thresholdTape_output, thresh, 255, THRESH_BINARY );
  threshold( greenHueImage, thresholdBall_output, thresh, 255, THRESH_BINARY );
  /// Find contoursBox



  /// Approximate contoursBox to polygons + get bounding rects and circles -------tape
  vector<vector<Point> > hullTape_poly( hullTape.size() );
  vector<Point2f>centerTape( hullTape.size() );
  vector<float>radiusTape( hullTape.size() );

  for( int i = 0; i < hullTape.size(); i++ ) { approxPolyDP( Mat(hullTape[i]), hullTape_poly[i], 3, true );
    boundRectTape[i] = boundingRect( Mat(hullTape_poly[i]) );
    minEnclosingCircle( (Mat)hullTape_poly[i], centerTape[i], radiusTape[i] );
  }


  /// Draw polygonal contour + bonding rects + circles
  for( int i = 0; i< hullTape.size(); i++ ) {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours( drawing, hullTape_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    bounding_rect=boundingRect(filteredContoursTape[i]); // Find the bounding rectangle for biggest contour
    rectangle( drawing, boundRectTape[i].tl(), boundRectTape[i].br(), color, 2, 8, 0 );
    circle( drawing, centerTape[i], (int)radiusTape[i], color, 2, 8, 0 );
  }

  /// Approximate contoursBox to polygons + get bounding rects and circles -------ball
  vector<vector<Point> > hullBall_poly( hullBall.size() );
  vector<Point2f>centerBall( hullBall.size() );
  vector<float>radiusBall( hullBall.size() );

  for( int i = 0; i < hullBall.size(); i++ ) {
    approxPolyDP( Mat(hullBall[i]), hullBall_poly[i], 3, true );
    boundRectBall[i] = boundingRect( Mat(hullBall_poly[i]) );
    minEnclosingCircle( (Mat)hullBall_poly[i], centerBall[i], radiusBall[i] );
  }


  /// Draw polygonal contour + bonding rects + circles
  for( int i = 0; i< hullBall.size(); i++ ) {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours( drawing, hullBall_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    bounding_rect=boundingRect(filteredContoursTape[i]); // Find the bounding rectangle for biggest contour
    rectangle( drawing, boundRectBall[i].tl(), boundRectBall[i].br(), color, 2, 8, 0 );
    circle( drawing, centerBall[i], (int)radiusBall[i], color, 2, 8, 0 );
  }


    

  //_____________________Center Calcs______(Calculates the center from Border Box, And calculates X,Y Offset)_______ Ok.. it's suppose to calculate from borderbox, but not yet. using hull instead
    // get the moments 

  // -------Ball
  vector<Moments> muBall(hullBall_poly.size());
  for( int i = 0; i<hullBall_poly.size(); i++ ) {
    muBall[i] = moments( hullBall_poly[i], false );
  }

  // get the centroid of figures.
  vector<Point2f> mcBall(hullBall_poly.size());
  for( int i = 0; i<hullBall_poly.size(); i++) {
    mcBall[i] = Point2f( muBall[i].m10/muBall[i].m00 , muBall[i].m01/muBall[i].m00 );
  }

  // draw filteredContours
  //Mat drawingcenter(canny_output.size(), CV_8UC3, Scalar(255,255,255));


  for( int i = 0; i<hullBall_poly.size(); i++ ) {
    Scalar color = Scalar(167,151,0); // B G R values
    //drawContours(drawing, hull_poly, i, color, 2, 8, hierarchy, 0, Point());
    circle( drawing, mcBall[i], 4, color, -1, 8, 0 );

    // offsets from centerBall
    Point centerBall = Point((mcBall[i].x), (mcBall[i].y));
    width_offset = width_goal - centerBall.x;
    height_offset = height_goal - centerBall.y;
    cout << "Offset From CenterBall x,y =" << height_offset << "," << width_offset << endl; //The output values... are a bit strange, need to look into that
    
  }

  // -------Tape

  vector<Moments> muTape(hullTape_poly.size());
  for( int i = 0; i<hullTape_poly.size(); i++ ) {
    muTape[i] = moments( hullTape_poly[i], false );
  }

  // get the centroid of figures.
  vector<Point2f> mcTape(hullTape_poly.size());
  for( int i = 0; i<hullTape_poly.size(); i++) {
    mcTape[i] = Point2f( muTape[i].m10/muTape[i].m00 , muTape[i].m01/muTape[i].m00 );
  }

  // draw filteredContours
  //Mat drawingcenter(canny_output.size(), CV_8UC3, Scalar(255,255,255));


  for( int i = 0; i<hullTape_poly.size(); i++ ) {
    Scalar color = Scalar(167,151,0); // B G R values
    //drawContours(drawing, hull_poly, i, color, 2, 8, hierarchy, 0, Point());
    circle( drawing, mcTape[i], 4, color, -1, 8, 0 );

    // offsets from centerTape
    Point centerTape = Point((mcTape[i].x), (mcTape[i].y));
    width_offset = width_goal - centerTape.x;
    height_offset = height_goal - centerTape.y;
    cout << "Offset From CenterTape x,y =" << height_offset << "," << width_offset << endl; //The output values... are a bit strange, need to look into that
  }
}

void Start() {
  std::thread processThread(&Run);
  processThread.detach();
}