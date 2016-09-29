//#include <opencv2/photo.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace cv;
using namespace std;

VideoCapture reader(string var)
{
  	return VideoCapture capLeft("var");
}