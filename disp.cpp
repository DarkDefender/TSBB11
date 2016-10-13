#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp> 
#include "disp.h"

using namespace cv;


Mat getDisparityMap(Ptr<StereoBM> sbmLeft, Ptr<StereoBM> sbmRight, Ptr<ximgproc::DisparityWLSFilter> sbmFilt,  Mat frameLeft, Mat frameRight){
	Mat dispLeft, dispRight, dispFilt;  
	// calculate disparity map
	sbmLeft->compute(frameLeft, frameRight, dispLeft, CV_64F);  
	sbmRight->compute(frameRight, frameLeft, dispRight, CV_64F);
	// filter
	sbmFilt->filter(dispLeft, frameLeft, dispFilt, dispRight);

	return dispFilt;
}
