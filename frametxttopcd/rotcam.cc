#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iomanip>
#include <fstream>
#include <Eigen/Geometry>
#include <Eigen/Core>
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
    while(fin >> t >> x >> y >> z >> q1 >> q2 >> q3 >> q4){
        frameNum.push_back(t);
        cQ.push_back(Eigen::Quaternion(q4,q1,q2,q3));
        cPos.push_back(Eigen::Vector3f(x,y,z));
		//cameraPoses.push_back(cv::Point3d(x,y,z));
	}
    
    fin.close();
    
    //calc normal
    Eigen::Matrix3Xf points;
    for( unsigned i = 0; i < cPos.size(); i++){
        points << cPos[i];
    }
    
    Eigen::Matrix<float, 3, 1> mean = points.rowwise().mean();
    const Eigen::Matrix3Xf points_centered = points.colwise() - mean;
    
    int setting = Eigen::ComputeThinU | Eigen::ComputeThinV;
    Eigen::JacobiSVD<Eigen::Matrix3Xf> svd = points_centered.jacobiSvd(setting);
    
    auto U = svd.matrixU();
    Eigen::Vector3d normal = U.col(2);
    normal.normalize();
    // find rot and translation
    Eigen::Vector3f xy_plane_normal_vector(0,1,0);
    Eigen::Vector3f rotation_vector = xy_plane_normal_vector.cross(normal);
    float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(normal));
    
    std::cout << "Rotation vector: "<< std::endl;
    std::cout << rotation_vector(0) << std::endl;
    std::cout << rotation_vector(1) << std::endl;
    std::cout << rotation_vector(2) << std::endl;
    std::cout << "Rotation angle: "<< std::endl;
    std::cout << theta << std::endl;
    
    Eigen::AngleAxisf rot(theta,rotation_vector/rotation_vector.norm());
    Eigen::Quaternionf quat(rot);
    
    //rotate
    //Eigen::
    
    //save to file
    ofstream fout;
	
    fout.open(fileout.c_str());
 	fout << fixed;
	
    for (unsigned int i; i < frameNum.size(); i++) {
        fout << frameNum[i];
        Eigen::Vector3f pos(cPos[i]*quat);
        fout << pos.x -mean.x<< pos.y - mean.y << pos.z - mean.z;
        Eigen::Quaternionf q(cQ[i]*quat);
        fout << q.w << q.x << q.y << q.z << std::endl;
        
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
