#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>

#ifndef PI
#define PI 3.141592653589793                
#endif

using namespace cv;
using namespace std;

#ifndef STEREOCAM_H

class stereoCam {
	private:
		// relative position and rotation in degrees
		//double x, y, z, yaw, pitch, roll; // relative position and rotation in degrees
		double type, fov_s, fov_h, k2, k3, k4, cx, cy, lx, ly; // camera type, field of view, radial distortion params, chip center, lens center
        Mat R, P, K;
		Point3f xyz_translation;
	public:
		stereoCam(char* fname); // constructor
		
		void read_cam_file(char* fname); // read .cam file with calibration
		Point3f getPosition(); // return relative position
		Mat getPose(); // return pose matrix
		Mat getIntrinsic(); // return camera intrinsics
		Mat getRotMat(); // return rotation matrix
		Point3f euler_from_rot(Mat rot); // return euler angles from rotation matrix
		Mat rot_from_euler(double yaw, double pitch, double roll); // return rot mat from euler ang.
		double f0_lens(double r1); // ?
		double f1_lens(double r1); // ?
		Point3f g_improj_to_xyz(double xs, double ys, double xi, double yi); // triangulate?
		Point3f g_xyz_to_improj(double xs, double ys, double x, double y, double z); // project to image plane?

};






#endif
