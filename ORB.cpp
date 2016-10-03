//
//  SIFT.cpp
//  
//
//  Created by Johan Lind on 2016-09-29.
//
//

#include "SIFT.hpp"
#include "stereoFeatures.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include <stdio.h>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"

using namespace cv::xfeatures2d;
using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    
    VideoCapture capLeft("left_512x384.avi");
    VideoCapture capRight("right_512x384.avi");
    Mat frameLeft, frameRight;
    capLeft >> frameLeft;
    capRight >> frameRight;
    
# Initiate ORB detector
     orb = ORB_create()
    
# find the keypoints with ORB
    kp = orb.detect(frameLeft,None)
    
# compute the descriptors with ORB
    kp, des = orb.compute(frameLeft, kp)
    
# draw only keypoints location,not size and orientation
    img2 = drawKeypoints(frameLeft,kp,color=(0,255,0), flags=0)
    plt.imshow(img2),plt.show()
    
    return 0;
}