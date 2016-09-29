#ifndef DISP_H
#define DISP_H

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;

Mat getDisparityMap(Ptr<StereoSGBM> sbm, Mat frameLeft, Mat frameRight);
#endif
