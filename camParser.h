
#ifndef CAMPARSER_H
#define CAMPARSER_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>                

#ifndef PI
#define PI 3.141592653589793                
#endif

using namespace cv;
using namespace std;


class camParser {
	private:
		// relative position and rotation in degrees
		//double x, y, z, yaw, pitch, roll; // relative position and rotation in degrees
		double type, fov_s, fov_h, k2, k3, k4, cx, cy, lx, ly; // camera type, field of view, radial distortion params, chip center, lens center
        Matx33d R, K;
		Mat P;
		Point3d xyz_translation;
	public:
		camParser(char* fname, int width, int height); // constructor
		
		void read_cam_file(char* fname, int width, int height); // read .cam file with calibration
		Point3d getPosition(); // return relative position
		Mat getPose(); // return pose matrix
		Matx33d getIntrinsic(); // return camera intrinsics
		Matx33d getRotMat(); // return rotation matrix
		Point3d euler_from_rot(Mat rot); // return euler angles from rotation matrix
		Mat rot_from_euler(double yaw, double pitch, double roll); // return rot mat from euler ang.
		double f0_lens(double r1); // ?
		double f1_lens(double r1); // ?
		Point3d g_improj_to_xyz(double width, double height, Point2d imgCoord); // triangulate?
		Point2d g_xyz_to_improj(double width, double height, Point3d X); // project to image plane?

};






#endif
