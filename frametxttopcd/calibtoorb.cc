#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <fstream>
#include <iomanip>
#include <vector>


int main(int argc, char** argv){
    if(argc != 4){
		std::cout << "Invalid number of input arguments given!" << std::endl
			<< "Usage: ./calibtoorb <calibration_file> <orb_slam_standard_settings> <output_file>" << std::endl;
		return -1;
	}
	/* read calibration and orb-slam settings */

	cv::FileStorage fs_calib(argv[1], cv::FileStorage::READ);
	cv::FileStorage orb_options(argv[2], cv::FileStorage::READ);
	
	cv::Mat K1,K2,D1,D2; // camera intrinsics
	cv::Mat	F,E,R,R1,R2,P1,P2,Q; // camera extrinsics
	std::vector<float> T; 
    /* read calibration settings */
	std::cout << "READING CAMERA PARAMETERS FROM " << argv[1] << std::endl;
	fs_calib["K1"] >> K1;
	fs_calib["K2"] >> K2;
	fs_calib["D1"] >> D1;
	fs_calib["D2"] >> D2;
	fs_calib["F"] >> F;
	fs_calib["E"] >> E;
	fs_calib["T"] >> T;
	fs_calib["R"] >> R;
	fs_calib["R1"] >> R1;
	fs_calib["R2"] >> R2;
	fs_calib["P1"] >> P1;
	fs_calib["P2"] >> P2;
	fs_calib["Q"] >> Q;

	std::cout << "READING CAM PROPS" << std::endl;
	int l_width, l_height, r_width, r_height;
	float fps;
	fs_calib["LEFT.width"] >> l_width;
	fs_calib["LEFT.height"] >> l_height;
	fs_calib["RIGHT.width"] >> r_width;
	fs_calib["RIGHT.height"] >> r_height;
	fs_calib["Camera.fps"] >> fps;
	/* read orb-slam settings */
	std::cout << "READING ORB_SLAM SETTINGS FROM " << argv[2] << std::endl;
	int ThDepth, orb_nFeatures, orb_nlevels, orb_iniThFast, orb_minThFast;
	float orb_scale;
	orb_options["ThDepth"] >> ThDepth;
    orb_options["ORBextractor.nFeatures"] >> orb_nFeatures;
	orb_options["ORBextractor.nLevels"] >> orb_nlevels;
    orb_options["ORBextractor.scaleFactor"] >> orb_scale;
    orb_options["ORBextractor.iniThFAST"] >> orb_iniThFast;
	orb_options["ORBextractor.minThFAST"] >> orb_minThFast;

    /* read orb-slam viewer settings */
	int v_width, v_psize, v_vpf, v_cwidth, v_vpx;
	float v_kfs, v_gwidth, v_csize, v_vpy, v_vpz;
	orb_options["Viewer.KeyFrameSize"] >> v_kfs;
	orb_options["Viewer.KeyFrameLineWidth"] >> v_width;
	orb_options["Viewer.GraphLineWidth"] >> v_gwidth;
	orb_options["Viewer.PointSize"] >> v_psize;
	orb_options["Viewer.CameraSize"] >> v_csize;
	orb_options["Viewer.CameraLineWidth"] >> v_cwidth;
	orb_options["Viewer.ViewpointX"] >> v_vpx;
	orb_options["Viewer.ViewpointY"] >> v_vpy;
	orb_options["Viewer.ViewpointZ"] >> v_vpz;
	orb_options["Viewer.ViewpointF"] >> v_vpf; 
	
	/* write settings and cailbration parameters to file */
	
	cv::FileStorage out(argv[3], cv::FileStorage::WRITE);
	std::cout << "WRINTING OUTPUT TO FILE " << argv[3] << std::endl;
    /* write calibration settings */
 	out << "K1" <<  K1;
 	out << "K2" <<  K2;
 	out << "D1" <<  D1;
 	out << "D2" <<  D2;
 	out << "F"  <<  F;
 	out << "E"  <<  E;
 	out << "T"  <<  T;
 	out << "R"  <<  R;
 	out << "R1" <<  R1;
 	out << "R2" <<  R2;
 	out << "P1" <<  P1;
 	out << "P2" <<  P2;
 	out << "Q"  <<  Q; 

	/* write orb-slam parameters */
	out << "LEFT_WIDTH" << l_width;
	out << "LEFT_HEIGHT" << l_height;
	out << "RIGHT_WIDTH" << r_width;
	out << "RIGHT_HEIGHT" << r_height;
 
	out << "Camera_width" << l_width;
	out << "Camera_height" << l_height;
	out << "Camera_fps" << fps;

	out << "Camera_fx" << P1.at<double>(0,0);
	out << "Camera_fy" << P1.at<double>(1,1);
	out << "Camera_cx" << P1.at<double>(0,2);
	out << "Camera_cy" << P1.at<double>(1,2);
 	
	out << "Camera_k1" << 0.0;
	out << "Camera_k2" << 0.0;
 	out << "Camera_p1" << 0.0;
 	out << "Camera_p2" << 0.0;

	out << "Camera_bf" << P2.at<double>(1,4);
	out << "Camera_RGB" << 0;
	out << "ThDepth" << ThDepth;
	
	out << "ORBextractor_nFeatures" << orb_nFeatures;
	out << "ORBextractor_scaleFactor" << orb_scale;
	out << "ORBextractor_nLevels" << orb_nlevels;

	out << "ORBextractor_iniThFAST" << orb_iniThFast;
	out << "ORBextractor_minThFAST" << orb_minThFast;

	/* Viewer Parameters */
	out << "Viewer_KeyFrameSize"		<< v_kfs;
	out << "Viewer_KeyFrameLineWidth"	<< v_width;
	out << "Viewer_GraphLineWidth"		<< v_gwidth;
	out << "Viewer_PointSize"			<< v_psize;
	out << "Viewer_CameraSize"			<< v_csize;
	out << "Viewer_CameraLineWidth"		<< v_cwidth;
	out << "Viewer_ViewpointX"			<< v_vpx;
	out << "Viewer_ViewpointY"			<< v_vpy;
	out << "Viewer_ViewpointZ"			<< v_vpz;
	out << "Viewer_ViewpointF"			<< v_vpf;

    return 0;
}	
/*
#--------------------------------------------------------------------------------------------
# Camera Parameters for ORB-SLAM. Adjust them!
#--------------------------------------------------------------------------------------------

# Camera calibration and distortion parameters (OpenCV) 
Camera.fx: 933.44407326729231 # =LEFT.P(0,0)
Camera.fy: 933.44407326729231 # =LEFT.P(1,1)
Camera.cx: 735.62555694580078 # =LEFT.P(0,2)
Camera.cy: 632.28582382202148 # =LEFT.P(1,2)

Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

Camera.width: 1536
Camera.height: 1088

# Camera frames per second 
Camera.fps: 20.0 #20.0

# stereo baseline times fx
Camera.bf: 395.3439947700014 # =RIGHT.P(1,4) (=norm(T)*fx)

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

# Close/Far threshold. Baseline times.
ThDepth: 35


 #--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 4000

# ORB Extractor: Scale factor between levels in the scale pyramid
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid
ORBextractor.nLevels: 12

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500 

*/
