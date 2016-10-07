#include <fstream>
#include <opencv2/opencv.hpp>
#include "camparse.h"
using namespace std;

#ifndef PI
#define PI 3.14159265
#endif

cv::Mat getRotationMat(char* fname){
	ifstream infile(fname);
	double x, y, z, yaw, pitch, roll;

	infile >> x >> y >> z >> yaw >> pitch >> roll;

	cv::Mat rot = YPRtoRot(yaw, pitch, roll);

	return rot;
}

cv::Mat YPRtoRot(double yaw, double pitch, double roll){
	double cy = cos(yaw*PI/180);
	double sy = sin(yaw*PI/180);
	double cp = cos(pitch*PI/180);
	double sp = sin(pitch*PI/180);
	double cr = cos(roll*PI/180);
	double sr = sin(roll*PI/180);

	cv::Mat yawMat = (cv::Mat_<double>(3,3)<< cy,sy,0, -sy,cy,0, 0,0,1);
 	cv::Mat pitchMat = (cv::Mat_<double>(3,3)<< cp,0,sp, 0,1,0, -sp,0,cp);    
 	cv::Mat rollMat = (cv::Mat_<double>(3,3)<< 1,0,0, 0,cr,-sr, 0,sr,cr);

	return rollMat*pitchMat*yawMat;

}


