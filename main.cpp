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
#include "disp.h"
//#include "akazefeature.h"




using namespace cv;
using namespace std;

int main(int argc, char* argv[]){

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
	Mat frameLeft, frameRight, fL, fR, disparityMap;

	// stereo blockmatch settings
	int ndisparities = 64;
	int SADWindowSize = 5;

	if(argc >= 3){
		ndisparities = atoi(argv[1]);
		SADWindowSize = atoi(argv[2]);
	}

	if(ndisparities % 16 !=0)
		ndisparities += 16-(ndisparities % 16);
	// init blockmatchers
	Ptr<StereoBM> sbmLeft = StereoBM::create(ndisparities, SADWindowSize);
	Ptr<ximgproc::DisparityWLSFilter> sbmWLSFilter = ximgproc::createDisparityWLSFilter(sbmLeft);
	Ptr<StereoMatcher> sbmRight = ximgproc::createRightMatcher(sbmLeft);
    
	sbmWLSFilter -> setLambda(1000);
	sbmWLSFilter -> setSigmaColor(1);
    if(argc >= 5){
		sbmWLSFilter->setLambda( atoi(argv[3]) );
		sbmWLSFilter->setSigmaColor( atoi(argv[4]) );
	}
	// open windows
	namedWindow("LeftFrame", CV_WINDOW_AUTOSIZE);
	namedWindow("RightFrame", CV_WINDOW_AUTOSIZE);
	namedWindow("disparity", CV_WINDOW_AUTOSIZE);
	namedWindow("featurematch", CV_WINDOW_AUTOSIZE);
	
	// feature matching
	vector<KeyPoint> kpts1, kpts2, matched1, matched2, inliers1, inliers2;
	Mat desc1, desc2;

	Ptr<AKAZE> akaze = AKAZE::create();

	vector<DMatch> goodMatches;

	vector< vector<DMatch> > NNMatches;

    BFMatcher matcher(NORM_HAMMING);

	float nn_match_ratio = 0.8f;
    int featureMatchingOn = 0;
	// fundamental matrix
	Mat fMatrix; // init to fundamental matrix for rect. stereo. 
	// main loop
	for(int frameCounter = 0; frameCounter < numFrames; frameCounter++){

        cout << "frame: " << frameCounter << endl;
		// get next frame 
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

			// get descriptors
			akaze->detectAndCompute(frameLeft, noArray(), kpts1, desc1);
			akaze->detectAndCompute(frameRight, noArray(), kpts2, desc2);

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
				/*Mat col = Mat::ones(3, 1, CV_64F);
				  col.at<double>(0) = matched1[i].pt.x;
				  col.at<double>(1) = matched1[i].pt.y;

				  col = homography * col;
				  col /= col.at<double>(2);
				  double dist = sqrt( pow(col.at<double>(0) - matched2[i].pt.x, 2) +
				  pow(col.at<double>(1) - matched2[i].pt.y, 2));
				  */
				//if(dist < inlier_threshold) {
				int new_i = static_cast<int>(inliers1.size());
				inliers1.push_back(matched1[i]);
				inliers2.push_back(matched2[i]);
				goodMatches.push_back(DMatch(new_i, new_i, 0));
				//}
			}

			Mat res;
			drawMatches(frameLeft, inliers1, frameRight, inliers2, goodMatches, res);
			imshow("featurematch", res);
			inliers1.clear();
			inliers2.clear();
			goodMatches.clear();
			matched1.clear();
			matched2.clear();
		}


        //disparityMap = getDisparityMap(sbmLeft, sbmRight, sbmWLSFilter, frameLeft, frameRight);
		Mat dispLeft, dispRight, dispFilt;  
		// calculate disparity map
		sbmLeft->compute(fL, fR, dispLeft);  
		sbmRight->compute(fR, fL, dispRight);
		// filter
		sbmWLSFilter->filter(dispLeft, fL, dispFilt, dispRight);
        disparityMap = dispFilt.clone();
		
		
		
		
		
		
		Mat normalizedDisp;
		double maxVal, minVal;
		minMaxLoc(disparityMap, &minVal, &maxVal);
		cout << "max disp = " << maxVal << ", min disp = " << minVal << endl;
		disparityMap.convertTo(normalizedDisp, CV_8UC1, 255/(maxVal-minVal));
		// show frames
		imshow("disparity", normalizedDisp);
		imshow("RightFrame", frameRight);
		imshow("LeftFrame", frameLeft);
		// wait for draw and quit on esc
		if(waitKey() == 27){
			cout << "user escape" << endl;
			break;
		}
	}

	return 0;

}
