#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "disp.h"

using namespace cv;

Mat getDisparityMap(Ptr<StereoSGBM> sbm, Mat frameLeft, Mat frameRight){
	Mat fL, fR, disparityMap;
   	double minVal, maxVal;
	cvtColor(frameLeft, fL, COLOR_BGR2GRAY); //.convertTo(fL, CV_8UC1);
      	cvtColor(frameRight, fR, COLOR_BGR2GRAY); //.convertTo(fR, CV_8UC1);
      
      	// calculate disparity map
      	sbm->compute(fR, fL, disparityMap);
      	minMaxLoc( disparityMap, &minVal, &maxVal );
      
      	// normalize disparity map
      	disparityMap.convertTo(disparityMap, CV_8UC1, 255/(maxVal - minVal));

	return disparityMap;
}
