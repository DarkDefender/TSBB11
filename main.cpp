//#include <opencv2/photo.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>

// include own headers
#include "disp.h"




using namespace cv;
using namespace std;

int main(int argc, char* argv[]){
  
  // init frames and vids
  VideoCapture capLeft, capRight;
  capLeft.open("data/left_512x384.avi");
  capRight.open("data/right_512x384.avi");
  Mat frameLeft, frameRight, fL, fR, disparityMap;
  
  // stereo blockmatch settings
  int ndisparities = 64;
  int SADWindowSize = 5;

  if(argc == 3){
	ndisparities = atoi(argv[1]);
	SADWindowSize = atoi(argv[2]);
  }

  Ptr<StereoSGBM> sbm = StereoSGBM::create(0, ndisparities, SADWindowSize);
  
  // get frame properties
  //double minVal, maxVal;
  
  double dWidth = capLeft.get(CV_CAP_PROP_FRAME_WIDTH);
  double dHeight = capLeft.get(CV_CAP_PROP_FRAME_HEIGHT);
  //  dfps = capLeft.get(CV_CAP_PROP_FRAME_RATE);
  cout << "framesize = " << dWidth << " x " << dHeight << endl;

  // open windows
  namedWindow("LeftFrame", CV_WINDOW_AUTOSIZE);
  namedWindow("RightFrame", CV_WINDOW_AUTOSIZE);
  namedWindow("disparity", CV_WINDOW_AUTOSIZE);
  namedWindow("3dpoints", CV_WINDOW_AUTOSIZE);

  // main loop
  while(1){

      // get next frame
      capLeft >> frameLeft;
      capRight >> frameRight;

      if (frameLeft.empty() || frameRight.empty()){
	  cout << "cannot read frame" << endl;
	  break;
	}

      disparityMap = getDisparityMap(sbm, frameLeft, frameRight);
      /*
      cvtColor(frameLeft, fL, COLOR_BGR2GRAY); //.convertTo(fL, CV_8UC1);
      cvtColor(frameRight, fR, COLOR_BGR2GRAY); //.convertTo(fR, CV_8UC1);
      
      // calculate disparity map
      sbm->compute(fR, fL, disp_map);
      minMaxLoc( disp_map, &minVal, &maxVal );
      
      // normalize disparity map
      disp_map.convertTo(frame_disp, CV_8UC1, 255/(maxVal - minVal));
      */
      // show frames
      imshow("disparity", disparityMap);
      imshow("LeftFrame", frameLeft);
      imshow("RightFrame", frameRight);

      // wait for draw and quit on esc
      if(waitKey(30) == 27){
	  cout << "user escape" << endl;
	  break;
	}
    }

  return 0;

}
