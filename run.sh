#!/bin/bash

if (( $# != 3 )); then
    echo "Illegal number of parameters"
	echo "Usage:"
	echo "$0 <left_img_seq>.avi <right_img_seq>.avi <opencv_cam_calib>.yaml"
	exit 2
fi

echo $1
echo $2
echo $3

#TODO Make sure we handle absolute input paths correctly

##exit 0

#Preprocess input image data

mkdir -p data/left
mkdir -p data/right

ffmpeg -i $1 data/left/%04d.png

ffmpeg -i $2 data/right/%04d.png

for filename in data/left/*.png; do
	base=$(basename $filename) 
	echo ${base%.*} >> data/timestamps.txt
done

Run orb slam 2 to extract camera path

cd ORB_SLAM2

./Examples/Stereo/stereo_saab Vocabulary/ORBvoc.txt ../$3 ../data/left ../data/right ../data/timestamps.txt

mv CameraTrajectory.txt ../data
mv KeyFrameTrajectory.txt ../data

cd ..

#Rectify keyframe images

mkdir -p data/lrect
mkdir -p data/rrect

while IFS='' read -r line || [[ -n "$line" ]]; do
    number=$(echo "$line" | awk '{print $1;}')
	printf -v number "%04d" ${number%.*}
#	echo $number
	fname=$number.png
	stereo-calibration/build/undistort_rectify -l data/left/"$fname" -r data/right/"$fname" -c $3 -L data/lrect/"$fname" -R data/rrect/"$fname"
done < "data/KeyFrameTrajectory.txt"

#Create disp files
mkdir -p data/disp
cd data/disp

for filename in ../lrect/*.png; do
	base=$(basename $filename) 
	../../spsstereo/build/spsstereo ../lrect/$base ../rrect/$base  
	mv ${base%.*}_left_disparity.png $base
done

cd ../../

#Create PCD files
mkdir -p data/pcd/
cd data/pcd

for filename in ../disp/*.png; do 
	base=$(basename $filename) 
	../../3drecon/disp2cloud ../left/$base $filename $3
done

cd ../../

#Merge all PCDs

cp pcdfile/emptyCloud.pcd finalCloud.pcd

frametxttopcd/build/frametxt2pcd data/KeyFrameTrajectory.txt camerapos.pcd

while IFS='' read -r line || [[ -n "$line" ]]; do
    number=$(echo "$line" | awk '{print $1;}')
	trans=$(echo "$line"  | awk '{$1=""; print $0}')
	printf -v number "%04d" ${number%.*}
#	echo $number
	echo $trans
	pcdname=$number.pcd
	build/cutting2 data/pcd/$pcdname finalCloud.pcd camerapos.pcd $trans
done < "data/KeyFrameTrajectory.txt" 

#build/cleanUp finalCloud.pcd 0.01 15 0.01

#Mesh final PCD


#Open blender to calc volume
