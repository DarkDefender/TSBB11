//#include <opencv2/photo.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
  // add frames and vids
  VideoCapture capLeft("left_512x384.avi");
  VideoCapture capRight("right_512x384.avi");
  Mat frameLeft, frameRight, fL, fR, disp_map, frame_disp;
  // add disparity
  int ndisparities = 64;
  int SADWindowSize = 5;

  if(argc == 2){
	ndisparities = atoi(argv[1]);
	SADWindowSize = atoi(argv[2]);
  }

  Ptr<StereoSGBM> sbm = StereoSGBM::create(2, ndisparities, SADWindowSize);
  double minVal, maxVal;
  
  double dWidth = capLeft.get(CV_CAP_PROP_FRAME_WIDTH);
  double dHeight = capLeft.get(CV_CAP_PROP_FRAME_HEIGHT);
  //  dfps = capLeft.get(CV_CAP_PROP_FRAME_RATE);
  cout << "framesize = " << dWidth << " x " << dHeight << endl;

  namedWindow("LeftFrame", CV_WINDOW_AUTOSIZE);
  namedWindow("RightFrame", CV_WINDOW_AUTOSIZE);
  namedWindow("disparity", CV_WINDOW_AUTOSIZE);
  namedWindow("3dpoints", CV_WINDOW_AUTOSIZE);
  while(1)
    {

      capLeft >> frameLeft;
      capRight >> frameRight;

      if (frameLeft.empty() || frameRight.empty())
	{
	  cout << "cannot read frame" << endl;
	  break;
	}


      cvtColor(frameLeft, fL, COLOR_BGR2GRAY);//.convertTo(fL, CV_8UC1);
      cvtColor(frameRight, fR, COLOR_BGR2GRAY);//.convertTo(fR, CV_8UC1);
      //minMaxLoc(frameLeft, &minVal, &maxVal);
      //minMaxLoc(fL, &minVal, &maxVal);
      
      sbm->compute(fR, fL, disp_map);
      minMaxLoc( disp_map, &minVal, &maxVal );
      //cout << "framedisparity  min: " << minVal << ", max: " << maxVal << endl;

      disp_map.convertTo(frame_disp, CV_8UC1, 255/(maxVal - minVal));

      imshow("disparity", frame_disp);
      
      imshow("LeftFrame", fL);
      imshow("RightFrame", fR);

      if(waitKey(30) == 27)
	{
	  cout << "user escape" << endl;
	  break;
	}
    }

  return 0;

}
