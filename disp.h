#ifndef DISP_H
#define DISP_H

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
using namespace cv;

Mat getDisparityMap(Ptr<StereoBM> sbmLeft, Ptr<StereoMatcher> sbmRight, Ptr<ximgproc::DisparityWLSFilter> sbmFilt, Mat frameLeft, Mat frameRight);
#endif
