#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iomanip>
#include <fstream>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
using namespace std;


void SaveCameraTrajectoryToPCD(const string &filein, const string &fileout){
	ifstream fin;
	fin.open(filein.c_str());
    vector<Eigen::Quaternionf> cQ;
    vector<Eigen::Vector3f> cPos;
    //vector<cv::Point3f> cameraPoses;
    vector<int> frameNum;
    
    float t,x,y,z,q1,q2,q3,q4;
	
    // read file
	std::cout << "reading file:" << filein << std::endl;

    while(fin >> t >> x >> y >> z >> q1 >> q2 >> q3 >> q4){
        frameNum.push_back(t);
        cQ.push_back(Eigen::Quaternionf(q4,q1,q2,q3));
        cPos.push_back(Eigen::Vector3f(x,y,z));

		//cout << t << " " << x << " " << y << " " << z << " " << q1 << " " << q2 << " " << q3  << " " << q4 << endl;
		//cameraPoses.push_back(cv::Point3d(x,y,z));
	}
	//cout << "Number of poses found = " << frameNum.size() << endl;
    
    fin.close();
    
    //calc normal
	//std::cout << "reading cam positions to matrix" << std::endl;
    Eigen::MatrixXf points(3, cPos.size());
    for(size_t i = 0; i < cPos.size(); i++){
		Eigen::Vector3f p = cPos[i];
        //cout << i << ": " << p << endl;
		
		points(0,i) = p(0);
		points(1,i) = p(1);
		points(2,i) = p(2); //<< cPos[i]; //(0), cPos[i](1), cPos[i](2);
    }
    /*cout << "number of points = " << endl;
	cout << points.rows() << "x" <<points.cols() << endl;
	cout << "matrix content = " << endl << points << endl;

	std::cout << "Mean = "; // << std::endl;
    */

	Eigen::Matrix<float, 3, 1> mean = points.rowwise().mean();
	Eigen::Translation3f trans(-mean);
    //cout << mean << endl;

	const Eigen::Matrix3Xf points_centered = points.colwise() - mean;
    
    int setting = Eigen::ComputeFullU | Eigen::ComputeThinV;
    Eigen::JacobiSVD<Eigen::Matrix3Xf> svd = points_centered.jacobiSvd(setting);
    
	//std::cout << "extracting normal = "; // << std::endl;
	Eigen::Matrix3f U = svd.matrixU();
    Eigen::Vector3f normal = U.col(2);
    normal.normalize();
	//cout << normal << endl;
    
	// find rot and translation
    Eigen::Vector3f yhat(0,1,0);
	Eigen::Quaternionf quat_rot = Eigen::Quaternionf::FromTwoVectors(normal, yhat);
    //cout << "Quat = " << quat_rot << endl;
    /*
    Eigen::Vector3f rotation_vector = xy_plane_normal_vector.cross(normal);
    rotation_vector.normalize();
	float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(normal));
    
    std::cout << "Rotation vector: "<< std::endl;
    std::cout << rotation_vector << std::endl;
    //std::cout << rotation_vector(1) << std::endl;
    //std::cout << rotation_vector(2) << std::endl;
    std::cout << "Rotation angle: "<< std::endl;
    std::cout << theta << std::endl;
    
    Eigen::AngleAxisf rot(theta,rotation_vector/rotation_vector.norm());
    Eigen::Quaternionf quat(rot);
    */
    //rotate
    //Eigen::
    
    //save to file
    ofstream fout;
	
    fout.open(fileout.c_str());
 	fout << fixed;
	fout << setprecision(9);
	cout << "writing to file " << fileout << endl;
    for (size_t i = 0; i < frameNum.size(); i++) {
        fout << frameNum[i] << " ";
        Eigen::Vector3f pos(cPos[i]);
		Eigen::Vector3f newPos;
		newPos = quat_rot*trans*pos;
        fout << newPos(0) << " " << newPos(1) << " " << newPos(2) << " ";
        Eigen::Quaternionf q(quat_rot*cQ[i]);
        fout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        //cout << frameNum[i] << " " << newPos(0) << " " << newPos(1) << " " << newPos(2) << endl;
    }
    
    
    /* print pcd header to file
	   
    fout << "VERSION .7" << std::endl
		<< "FIELDS x y z" << std::endl
		<< "SIZE 4 4 4" << std::endl
		<< "TYPE F F F" << std::endl
		<< "COUNT 1 1 1" << std::endl
		<< "WIDTH " << cameraPoses.size() << std::endl
		<< "HEIGHT 1" << std::endl
		<< "VIEWPOINT 0 0 0 1 0 0 0" << std::endl
		<< "POINTS " << cameraPoses.size() << std::endl
		<< "DATA ascii" << std::endl;
	loop through all points and print to file

    for(unsigned i = 0; i < cameraPoses.size(); i++){
		cv::Point3f p = cameraPoses.at(i);
		fout << setprecision(9);
		fout << p.x << " " << p.y << " " << p.z << " " << endl;
	}
     */
	fout.close();
}

int main(int argc, char** argv){
	SaveCameraTrajectoryToPCD(argv[1], argv[2]);

	return 0;
}
