#include <fstream>
#include <opencv2/opencv.hpp>
#include "camparse.h"
using namespace std;

#ifndef PI
#define PI 3.141592653589793
#endif

cv::Mat getRotationMat(char* fname){
	ifstream infile(fname);
	double x, y, z, yaw, pitch, roll;

	infile >> x >> y >> z >> yaw >> pitch >> roll;

	// make translation vector and rotation matrix
	cv::Mat translation = (cv::Mat_<double>(1,3) << x, y, z);
	cv::Mat rot = YPRtoRot(yaw, pitch, roll);

    // flip row-col, append translation, and flip back
	rot = rot.t();
	rot.push_back(translation);
	rot = rot.t();

	return rot;

	/* Todo:
	 * add parsing for translation to pose matrix P = [R, t];
	 */

    
	/*
	ifstream infile("data/cam_01.cam");
	double x, y, z, yaw, pitch, roll, type, fov_s, fov_h, k2, k3, k4, cx, cy, lx, ly;
	infile >> x >> y >> z >> yaw >> pitch >> roll >> type >> fov_s >> fov_h >> k2 >> k3 >> k4 >> cx >> cy >> lx >> ly; 
	*/
}


cv::Mat YPRtoRot(double yaw, double pitch, double roll){
	/*
	 * transforms euler angles in degrees to Rotation matrix
	 */

	double cy = cos(yaw*PI/180);
	double sy = sin(yaw*PI/180);
	double cp = cos(pitch*PI/180);
	double sp = sin(pitch*PI/180);
	double cr = cos(roll*PI/180);
	double sr = sin(roll*PI/180);

	cv::Mat yawMat = (cv::Mat_<double>(3,3)<< cy,-sy,0, sy,cy,0, 0,0,1);
 	cv::Mat pitchMat = (cv::Mat_<double>(3,3)<< cp,0,sp, 0,1,0, -sp,0,cp);    
 	cv::Mat rollMat = (cv::Mat_<double>(3,3)<< 1,0,0, 0,cr,-sr, 0,sr,cr);

	cout << yawMat << endl << endl;
 	cout << pitchMat << endl << endl;
 	cout << rollMat << endl << endl;   
	return rollMat.t()*pitchMat.t()*yawMat.t();

}
/*
cv::Mat getIntrinsics(char* fname){
 	ifstream infile(fname);
	double x, y, z, yaw, pitch, roll, type, fov_s, fov_h, k2, k3, k4, cx, cy, lx, ly;

	infile >> x >> y >> z >> yaw >> pitch >> roll >> type >> fov_s >> fov_h >> k2 >> k3 >> k4 >> cx >> cy >> lx >> ly;
 
}
*/

/*
StereoRigParser::StereoRigParser(int a) {
cout << a;
};

void StereoRigParser::setCamLeft() {
};
*/
