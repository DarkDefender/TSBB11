
#include <opencv2/opencv.hpp>
#include "camParser.h"
#include <vector>
#include <fstream>
#include <cmath>

#ifndef PI
#define PI 3.141592653589793                
#endif

using namespace cv;
using namespace std;

// constructor
camParser::camParser(char* fname, int width, int height){
	// default constructor
	camParser::read_cam_file(fname, width, height);
}
// parse camfile
void camParser::read_cam_file(char* fname, int width, int height){
	ifstream infile(fname);
	double x, y, z, yaw, pitch, roll;// type, fov_s, fov_h, k2, k3, k4, cx, cy, lx, ly;
	infile >> x >> y >> z >> yaw >> pitch >> roll >> type >> fov_s >> fov_h >> k2 >> k3 >> k4 >> cx >> cy >> lx >> ly; 
	
	// K-matrix
	double fx = width*0.5/tan(0.5*PI*fov_h/180); // k11
	double fy = height*0.5/tan(0.5*PI*fov_s/180); // k22
    // img-center
	Point2d U0 = g_xyz_to_improj(1534, 1152, Point3d(1, cy, cx));
	double u0 = U0.x;
	double v0 = U0.y;

	K << fx,0,u0, 0,fy,v0, 0,0,1;

	cout << fx << ", " << fy << " focalllllll " << endl;
	cout << "center= " << U0;
	//update rel. position
	// xyz_translation = t;
	xyz_translation.x = x;
	xyz_translation.y = y;
	xyz_translation.z = z;

	// update matrices
	R = rot_from_euler(yaw, pitch, roll);
	P = Mat(R);
	P = P.t();

    Mat xyz = (Mat_<double>(1,3) << x, y, z);
	P.push_back(xyz);
	P = P.t();
}

// getters
Point3d camParser::getPosition(){
	return xyz_translation; 
}

Mat camParser::getPose(){
	return P;
}

Matx33d camParser::getIntrinsic(){
	return K;
}

Matx33d camParser::getRotMat(){
	return R;
}

// computers
Point3d camParser::euler_from_rot(Mat rot){
	/*euler_from_rot_mat
	  function [yaw, pitch, roll] = euler_from_rot(R)

	  yaw = atan2(R(1,2), R(1,1));
	  pitch = asin(-R(1,3));
	  roll = atan2(R(2,3), R(3,3));
	  */
    double yaw, pitch, roll;
	Point3d euler_angles;
	yaw = 180/PI*atan2( rot.at<double>(0,1), rot.at<double>(0,0) );
	pitch = 180/PI*asin( -rot.at<double>(0,2) );
	roll = atan2( rot.at<double>(1,2), rot.at<double>(2,2) )*180/PI; 
	
	euler_angles.x = yaw;
	euler_angles.y = pitch;
	euler_angles.z = roll;

	return euler_angles;
}

Mat camParser::rot_from_euler(double yaw, double pitch, double roll){
	/*
	 * transforms euler angles in degrees to Rotation matrix
	 */

	/*
	*/
	double cy = cos(yaw*PI/180);
	double sy = sin(yaw*PI/180);
	double cp = cos(pitch*PI/180);
	double sp = sin(pitch*PI/180);
	double cr = cos(roll*PI/180);
	double sr = sin(roll*PI/180);

	Mat yawMat = (cv::Mat_<double>(3,3)<< cy,-sy,0, sy,cy,0, 0,0,1);
	Mat pitchMat = (cv::Mat_<double>(3,3)<< cp,0,sp, 0,1,0, -sp,0,cp);    
	Mat rollMat = (cv::Mat_<double>(3,3)<< 1,0,0, 0,cr,-sr, 0,sr,cr);

	return rollMat.t()*pitchMat.t()*yawMat.t();

} 


double camParser::f0_lens(double radius1){
	/* 
	// undistort
	function res = F0_LENS(proj, r1)
	
	res = (1.0 + (proj.k2 + (proj.k3 + proj.k4 * r1) * r1) * r1 * r1);
	
	% F0_LENS(proj, r1) F0_LENSK(proj->k2, proj->k3, proj->k4, r1)
	% F0_LENSK(k2, k3, k4, r) (1.0 + (k2 + (k3 + k4 * r) * r) * r * r)
	
	*/
	return (1.0 + (k2 + (k3 + k4*radius1)*radius1)*radius1*radius1);
}

double camParser::f1_lens(double radius1){
	/* 
	   function res = F1_LENS(proj, r1)
	   
	   res  = (1.0 + ((3.0*proj.k2) + ((4.0*proj.k3) + (5.0*proj.k4) * r1) * r1) * r1 * r1);
	   
	   % F1_LENS(proj, r1) F1_LENSK(proj->k2, proj->k3, proj->k4, r1)
	   % F1_LENSK(k2, k3, k4, r) F0_LENSK(3.0 * k2, 4.0 * k3, 5.0 * k4, r)
	   % F0_LENSK(k2, k3, k4, r) (1.0 + (k2 + (k3 + k4 * r) * r) * r * r)
	*/
	return (1.0 + ((3.0*k2) + ((4.0*k3) + (5.0*k4)*radius1)*radius1)*radius1*radius1);
}


Point3d camParser::g_improj_to_xyz(double width, double height, Point2d imgPoint){
	/*
	 * return xyz normalized image coordinates
	 * !!! DISCLAIMER: SAAB defines x - forward, y - right, z - down, from image
	 * normally x - right, y - down, and z - forward!!!!
	 */
	double xs = width, ys = height;
	double xi = imgPoint.x, yi = imgPoint.y;
	Point3d X;
	double xg,yg,zg,r1,r2,c1;
	double GPIX0 = 0.5;
	xi+=GPIX0;
	yi+=GPIX0;
	
	//if(type == 0){ // necessary to check?
		xg = 1;
		yg = (xi/xs - 0.5)*tan(PI*fov_s*0.5/180)*2 - cx;
		zg = (yi/ys - 0.5)*tan(PI*fov_h*0.5/180)*2 - cy;

		r2 = sqrt(zg*zg + yg*yg);
		c1 = 1.0;
		for( int i = 0; i < 5; i++){
			r1 = c1*r2;
			c1 = c1 - (c1*f0_lens(r1) - 1.0)/f1_lens(r1);
		}

		yg = c1*yg;
		zg = c1*zg;

		yg = yg-lx;
		zg = zg-ly;

        // saab default:
		X.x = xg;
		X.y = yg;
		X.z = zg;
        // normal xyz:
		// XG.x = yg; XG.y = zg; XG.z = xg;
		return X;
	//}
	
	
	/* // project 3D (?)
	   function [xg, yg, zg] = g_improj_to_xyz(proj, xs, ys, xi, yi)
	   
	   GPIX0 = 0.5;
	   
	   xi = xi + GPIX0;
	   yi = yi + GPIX0;
	   
	   % Matlab starts with 1,1 (assumes Matlab im coords as input
	   xi = xi - 1.0;
	   yi = yi - 1.0;
	   
	   if proj.type == 0 %G_PT_LENS
	   
	   x = 1.0;
	   y = (xi / xs - 0.5) * tan(proj.fov_s * 0.5) * 2.0 - proj.cx;
	   z = (yi / ys - 0.5) * tan(proj.fov_h * 0.5) * 2.0 - proj.cy;
	   
	   r2 = sqrt(z*z + y*y);
	   c1 = 1.0;
	   
	   for i = 1:5
	   r1 = c1*r2;
	   c1 = c1 - (c1 * F0_LENS(proj, r1) - 1.0) / F1_LENS(proj, r1);
	   end
	   
	   y = c1 * y;
	   z = c1 * z;
	   
	   y = y - proj.lx;
	   z = z - proj.ly;
	   
	   xg = x;
	   yg = y;
	   zg = z;
	   
	   
	   end

*/
	// triangulate?
}

Point2d camParser::g_xyz_to_improj(double width, double height, Point3d X){
	/*
	 * !!! USES SAAB-ORIENTATION
	 */
	double xs = width, ys = height;
	double x = X.x, y = X.y, z = X.z;
	double xi,yi,wi,GPIX0 = 0.5,w,r1,c2;
	Point2d imgCoord;

	w = 1.0/x;
	y = y*w;
	z = z*w;

	y = y + lx;
	z = z + ly;

	r1 = sqrt(z*z + y*y);
	c2 = f0_lens(r1);

	y = c2*y;
	z = c2*z;

	xi = ((y+cx)*0.5 / tan(PI*fov_s*0.5/180) + 0.5) * xs;
	yi = ((z+cy)*0.5 / tan(PI*fov_h*0.5/180) + 0.5) * ys;
	wi = w;

	xi = xi - GPIX0;
	yi = yi - GPIX0;
	imgCoord.x = xi/wi;
	imgCoord.y = yi/wi;

	return imgCoord;

	/* // project 3D to img plane
	   function [xi, yi, wi] = g_xyz_to_improj(proj, xs, ys, x, y, z)
	   
	   GPIX0 = 0.5;
	   
	   if proj.type == 0 %G_PT_LENS
	   
	   w = 1.0/x;
	   y = y * w;
	   z = z * w;
	   
	   y = y + proj.lx;
	   z = z + proj.ly;
	   
	   r1 = sqrt(z*z + y*y);
	   c2 = F0_LENS(proj, r1);
	   
	   y = c2 * y;
	   z = c2 * z;
	   
	   xi = ((y + proj.cx) * 0.5 / tan(proj.fov_s * 0.5) + 0.5) * xs;
	   yi = ((z + proj.cy) * 0.5 / tan(proj.fov_h * 0.5) + 0.5) * ys;
	   wi = w;
	   
	   
	   
	   end
	   
	   xi = xi - GPIX0;
	   yi = yi - GPIX0;
	   
	   % Matlab starts with (1,1)
	   xi = xi + 1.0;
	   yi = yi + 1.0;
	   
	   

*/
	// project to image plane?
}


