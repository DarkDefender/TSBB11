#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
 
//#include "opencv2/ximgproc/disparity_filter.hpp"

using namespace cv;
//using namespace cv::ximgproc;

int main(int argc, char** argv){
	if (argc != 3){
		std::cout << "Invalid number of input arguments!" << std::endl
			<< "Usage: " << argv[0] << "<left_rectified>.png <right_rectified>.png" << std::endl;
		return -1;
	}
    Mat disp;
	Mat imgL = imread(argv[1], IMREAD_GRAYSCALE);
	Mat imgR = imread(argv[2], IMREAD_GRAYSCALE);

	if( imgL.cols == 0 || imgR.cols == 0 || imgL.rows == 0 || imgR.cols == 0
			|| imgL.cols != imgR.cols || imgL.rows != imgR.rows ){
		std::cout << "Input image empty or of different sizes!" << std::endl;
		return -1;
	}
    /* stereosgbm settings */
	int window_size = 7;
	int min_disp = 0;
	int num_disp = 256-min_disp;
	int uniquenessRatio = 10;
	int speckleWindowSize = 21;
	int speckleRange = 32;
	int disp12MaxDiff = 1;
	int preFilterCap = 63;
	int P1 = 8*1*window_size*window_size;
	int P2 = 32*1*window_size*window_size;

    // StereoSGBM::create (int minDisparity, int numDisparities, int blockSize, int P1=0, int P2=0, 
	//		int disp12MaxDiff=0, int preFilterCap=0, int uniquenessRatio=0, int speckleWindowSize=0, int speckleRange=0, int mode=StereoSGBM::MODE_SGBM)
	Ptr<StereoSGBM> sbm = StereoSGBM::create(min_disp, num_disp, window_size, P1, P2,
			disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, StereoSGBM::MODE_HH);
    
    sbm->compute(imgL, imgR, disp);
	// normalize disparity map
	disp = disp/16.0;

	// write to file
	std::string outputBaseFilename = argv[1];
	size_t slashPosition = outputBaseFilename.rfind('/');
	if (slashPosition != std::string::npos) outputBaseFilename.erase(0, slashPosition+1);
	size_t dotPosition = outputBaseFilename.rfind('.');
	if (dotPosition != std::string::npos) outputBaseFilename.erase(dotPosition);
	std::string outputDisparityImageFilename = outputBaseFilename + "_left_disparity.png";
	
	imwrite(outputDisparityImageFilename, disp);
	
	return 0;


}
