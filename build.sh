#!/bin/bash
# build 3d dense reprojection
cd 3drecon
cmake ./OpenCVReprojectImageToPointCloud
make
cd ..
# build ORB-SLAM
unamestr=`uname`
if [[ "$unamestr" != 'Darwin' ]]; then
	cd ORB_SLAM2
	sh build.sh
	cd ..
fi
# build calibration
cd stereo-calibration
mkdir build
cd build
cmake ../
make
cd ../../
# build spsstereo
cd spsstereo
mkdir build
cd build
cmake ../
make
cd ../../
# build cameratrajectory to pcd converter
cd misc
mkdir build
cd build
cmake ../
make
cd ../../
# build point cloud merger
cd pcd_progs
mkdir build
cd build
cmake ../
make
cd ../../
# build disparity map
cd dispmap
mkdir build
cd build
cmake ..
make
cd ../../
# build texture mapping
cd mesh
mkdir build
cd build
cmake ..
make
cd ../../

