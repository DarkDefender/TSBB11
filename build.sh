#!/bin/bash
cd 3drecon
cmake ./OpenCVReprojectImageToPointCloud
make
cd ..

cd ORB_SLAM2
sh build.sh
cd ..

cd stereo-calibration
mkdir build
cd build
cmake ../
make
cd ../../

cd spsstereo
mkdir build
cd build
cmake ../
make
cd ../../

mkdir build
cd build
cmake ../

