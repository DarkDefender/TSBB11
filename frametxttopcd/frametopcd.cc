#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>
#include <iomanip>
#include <fstream>

using namespace std;


void SaveCameraTrajectoryToPCD(const string &filein, const string &fileout){
	ifstream fin;
	fin.open(filein.c_str());
	vector<cv::Point3f> cameraPoses;
	float t,x,y,z,q1,q2,q3,q4;
	while(fin >> t >> x >> y >> z >> q1 >> q2 >> q3 >> q4){
		cameraPoses.push_back(cv::Point3d(x,y,z));
	}
	fin.close();
    cout << cameraPoses.size();
	ofstream fout;
	fout.open(fileout.c_str());
 	fout << fixed;
	/* print pcd header to file */
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
	/* loop through all points and print to file */

    for(unsigned i = 0; i < cameraPoses.size(); i++){
		cv::Point3f p = cameraPoses.at(i);
		fout << setprecision(9);
		fout << p.x << " " << p.y << " " << p.z << " " << endl;
	}
	fout.close();
}

int main(int argc, char** argv){
	SaveCameraTrajectoryToPCD(argv[1], argv[2]);

	return 0;
}
