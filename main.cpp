//#include <opencv2/photo.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>

// include own headers
//#include "disp.h"
//#include "akazefeature.h"
#include "stereoCam.h"



using namespace cv;
using namespace std;
#ifndef PI
#define PI 3.141592653589793
#endif
int main(int argc, char* argv[]){
    // parse camera
	stereoCam cam0 = stereoCam("data/calib/cam_00.cam"); 
	stereoCam cam1 = stereoCam("data/calib/cam_01.cam");
   
    cout << "cam0 = " << endl; 
	Point3f pos0 = cam0.getPosition();
	cout << "pos = " << endl;
	cout << pos0 << endl;
	cout << "rot = " << endl << cam0.getRotMat() << endl << endl;
    
    cout << "cam1 = " << endl; 
	Point3f pos1 = cam1.getPosition();
	cout << "pos = " << endl;
	cout << pos1 << endl;
	cout  << "rot = " << endl << cam1.getRotMat() << endl << endl;
 
    cout << "testing cam..." << endl;
	double xi = 1500, yi = 1250.0;

	cout << "img point = " << yi << ", " << xi << endl;

	Point3f xyz = cam0.g_improj_to_xyz(1, 1, xi, yi);
	cout << "normalized = " << xyz << endl;
	Point3f xy = cam0.g_xyz_to_improj(1, 1, xyz.x, xyz.y, xyz.z);
	cout << "reprojected = " << xy << endl;

    return 0;

}
    /* testing
	Mat M = (Mat_<double>(3,3) << 1,1,1,2,2,2,3,3,3);
	Mat x = (Mat_<double>(1,3) << 4,4,4);
	M.push_back(x);
	cout << "M = " << endl << format(M, Formatter::FMT_NUMPY) << endl << endl;
	M.t();
    cout << "M = " << endl << format(M, Formatter::FMT_NUMPY) << endl << endl; 
	
   
	
	cout << "starting master project..." << endl;
	cout << argc << " input arguments given:" << endl;
	for( int i=0; i<argc; i++){
		cout << "	" << argv[i] << endl;
	}
	cout << endl << endl;

	// init frame grabbers
	VideoCapture capLeft, capRight;
	//capRight.open("data/ambush_5_right.jpg");
	//capLeft.open("data/ambush_5_left.jpg");
    capLeft.open("data/left_512x384.avi");
	capRight.open("data/right_512x384.avi"); 
 
    //capLeft.open("data/left_1536x1088.avi");
	//capRight.open("data/right_1536x1088.avi"); 
	
	if( !capLeft.isOpened() || !capRight.isOpened()){
		cout << "Can't open frames!" << endl;
        return -1;
	}
	
	// get and set video props properties
	int numFrames = capLeft.get(CV_CAP_PROP_FRAME_COUNT); 
	//capLeft.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	//capRight.set(CV_CAP_PROP_FORMAT, CV_8UC1);

	double dWidth = capLeft.get(CV_CAP_PROP_FRAME_WIDTH);
	double dHeight = capLeft.get(CV_CAP_PROP_FRAME_HEIGHT);

	// get crop-factor
	double nWidth = 1536;
	double nHeight = 1152;
	double cropFactorWidth = dWidth/nWidth;
	double cropFactorHeight = dHeight/nHeight;

    // print props
	cout << "Number of frames = " << numFrames << endl;
	cout << "Framesize = " << dWidth << " x " << dHeight << endl;
	cout << "Cropfactor: " << "width = " << cropFactorWidth;
 	cout << ", height = " << cropFactorHeight << endl;

    //init frames and mats
	Mat frameLeft, frameRight, fL, fR;


	// open windows
	namedWindow("LeftFrame", CV_WINDOW_AUTOSIZE);
	namedWindow("RightFrame", CV_WINDOW_AUTOSIZE);
	namedWindow("featurematch", CV_WINDOW_AUTOSIZE);
	
    int featureMatchingOn = 0;
	// feature matching
	if(argc >= 2)
		featureMatchingOn = atoi( argv[1] );
	if(featureMatchingOn){
		cout << "using feature matching" << endl;
	} else {
		cout << "not using feature matching" << endl;
	}

	
	vector<KeyPoint> kpts1, kpts2, matched1, matched2, inliers1, inliers2;
	Mat desc1, desc2;

	// choose detector, ORB by default
	Ptr<ORB> detector = ORB::create();
	
	if(argc >= 3){
		if( atoi( argv[2] ) ){
			//detector.release();
			Ptr<AKAZE> detector = AKAZE::create();
			cout << "Using AKAZE feature detector" << endl << endl;
		} else {
			cout << "Using ORB feature detector" << endl << endl;
		}
	}

	// init vectors for feature matching
	vector<DMatch> goodMatches;

	vector< vector<DMatch> > NNMatches;
	// init matcher
	BFMatcher matcher(NORM_HAMMING);

	const double nn_match_ratio = 0.8f;
	const double ransac_thresh = 2.5f;

	// fundamental matrix
	Mat fMatrix; // init to fundamental matrix for rect. stereo. 
	
	// main loop
	for(int frameCounter = 0; frameCounter < numFrames; frameCounter++){

        cout << "frame: " << frameCounter << endl;
		// get next view and convert to grayscale 
		capLeft >> frameLeft;
		capRight >> frameRight;
		cvtColor(frameLeft, fL, COLOR_BGR2GRAY);
 		cvtColor(frameRight, fR, COLOR_BGR2GRAY);  
		if (frameLeft.empty() || frameRight.empty()){
			cout << "cannot read frame" << endl;
			return -1;
		}
		
		// feature matching
		if (featureMatchingOn){
			// clear vectors
			inliers1.clear();
			inliers2.clear();
			goodMatches.clear();
			matched1.clear();
			matched2.clear(); 
			// get descriptors
			detector->detectAndCompute(fL, noArray(), kpts1, desc1);
			detector->detectAndCompute(fR, noArray(), kpts2, desc2);
			//orb->detectAndCompute(fL, noArray(), kpts1, desc1);
			//orb->detectAndCompute(fL, noArray(), kpts2, desc2);
			// match points
			matcher.knnMatch(desc1, desc2, NNMatches, 2);

			for(size_t i = 0; i < NNMatches.size(); i++) {
				DMatch first = NNMatches[i][0];
				float dist1 = NNMatches[i][0].distance;
				float dist2 = NNMatches[i][1].distance;

				if(dist1 < nn_match_ratio * dist2) {
					matched1.push_back(kpts1[first.queryIdx]);
					matched2.push_back(kpts2[first.trainIdx]);
				}
			}

			// check if points fulfil epipolar geometry (y1'*fMatrix*y2 = 0); 
			for(unsigned i = 0; i < matched1.size(); i++) {
				double dist = abs( matched1[i].pt.y - matched2[i].pt.y );
				
				if(dist < ransac_thresh) {
					int new_i = static_cast<int>(inliers1.size());
					inliers1.push_back(matched1[i]);
					inliers2.push_back(matched2[i]);
					goodMatches.push_back(DMatch(new_i, new_i, 0));
				}
			}

			Mat res;
			drawMatches(frameLeft, inliers1, frameRight, inliers2, goodMatches, res);
			imshow("featurematch", res);

			// triangulate 3D


		}

		// show frames
		imshow("RightFrame", frameRight);
		imshow("LeftFrame", frameLeft);
		// wait for draw and quit on esc
		if(waitKey() == 27){
			cout << "user escape" << endl;
			break;
		}
	}

	return 0;

}   */
