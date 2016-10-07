#include <opencv2/opencv.hpp>
using namespace std;

#ifndef PI
#define PI 3.14159265
#endif

cv::Mat getRotationMat(char* fname);

cv::Mat YPRtoRot(double yaw, double pitch, double roll);

