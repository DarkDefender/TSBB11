//#include <opencv2/photo.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <opencv2/viz.hpp>

// include own headers
#include "disp.h"
//#include "akazefeature.h"

using namespace cv;
using namespace std;

int main(int argc, char* argv[]){

	// init frames and vids
	VideoCapture capLeft, capRight;
	capLeft.open("data/left_512x384.avi");
	capRight.open("data/right_512x384.avi");
	Mat frameLeft, frameRight, fL, fR, disparityMap;

	// stereo blockmatch settings
	int ndisparities = 64;
	int SADWindowSize = 5;
    float baseFocalProd = 0.0025; //[m]  guessing....

	if(argc == 3){
		ndisparities = atoi(argv[1]);
		SADWindowSize = atoi(argv[2]);
	}

	Ptr<StereoSGBM> sbm = StereoSGBM::create(0, ndisparities, SADWindowSize);

	// get frame properties
	//double minVal, maxVal;

	double dWidth = capLeft.get(CV_CAP_PROP_FRAME_WIDTH);
	double dHeight = capLeft.get(CV_CAP_PROP_FRAME_HEIGHT);
	//  dfps = capLeft.get(CV_CAP_PROP_FRAME_RATE);
	cout << "framesize = " << dWidth << " x " << dHeight << endl;

	// open windows
	namedWindow("LeftFrame", CV_WINDOW_AUTOSIZE);
	namedWindow("RightFrame", CV_WINDOW_AUTOSIZE);
	namedWindow("disparity", CV_WINDOW_AUTOSIZE);
	//namedWindow("3dpoints", CV_WINDOW_AUTOSIZE);
	namedWindow("result", CV_WINDOW_AUTOSIZE);
	// feature matching
	vector<KeyPoint> kpts1, kpts2, matched1, matched2, inliers1, inliers2;
    vector<double> depth;
    vector<Mat> xCoord;
	Mat desc1, desc2;
    
   // Mat invK(3,3, DataType<double>::type);
   // invK = {1,0,0,0,1,0,0,0,1};
    Mat invK = (Mat_<float>(3,3) << 1,0,0,0,1,0,0,0,1);

	Ptr<AKAZE> akaze = AKAZE::create();

	vector<DMatch> goodMatches;

	vector< vector<DMatch> > NNMatches;

    BFMatcher matcher(NORM_HAMMING);

	float nn_match_ratio = 0.8f;
    int frameCounter = 0;
    int featureMatchingOn = 1;
	// fundamental matrix
	Mat fMatrix; // init to fundamental matrix for rect. stereo. 
	// main loop
	while(1){
		frameCounter++;
        cout << "frame: " << frameCounter << endl;
		// get next frame
		capLeft >> frameLeft;
		capRight >> frameRight;

		if (frameLeft.empty() || frameRight.empty()){
			cout << "cannot read frame" << endl;
			return -1;
		}
		
		if (featureMatchingOn){
		// feature matching
		Mat fL, fR;
		cvtColor(frameLeft, fL, COLOR_BGR2GRAY);
		cvtColor(frameRight, fR, COLOR_BGR2GRAY);

		// get descriptors
		akaze->detectAndCompute(fL, noArray(), kpts1, desc1);
		akaze->detectAndCompute(fR, noArray(), kpts2, desc2);

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
		drawMatches(fL, inliers1, fR, inliers2, goodMatches, res);
        imshow("result", res);
		inliers1.clear();
		inliers2.clear();
			goodMatches.clear();
			matched1.clear();
			matched2.clear();
		}
    
        disparityMap = getDisparityMap( sbm , frameLeft, frameRight);
		// show frames
		imshow("disparity", disparityMap);
		imshow("RightFrame", frameRight);
		imshow("LeftFrame", frameLeft);
		// wait for draw and quit on esc
		if(waitKey(30) == 27){
			cout << "user escape" << endl;
			break;
		}
        
        
        for (int i = 0; i<inliers1.size();i++){
            double deltaX = inliers1[i].pt.x - inliers2[goodMatches[i].queryIdx].pt.x;
            depth.push_back(baseFocalProd/deltaX);
            Mat homPoint = (Mat_<float>(3,1) << inliers1[i].pt.x,inliers1[i].pt.y,1);
            xCoord.push_back(invK*homPoint*depth[i]);
        }
        
	}
    

	return 0;

}
