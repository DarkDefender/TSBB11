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
RED='\033[1;31m'
GREEN='\033[1;32m'
NC='\033[0m' # No Color
printf "I ${RED}love${NC} Stack ${GREEN}Overflow${NC}\n"
#exit 0

if [ -d "data" ]; then
	echo "The data directory already exists. Do you wish to remove it?"
	echo "You must remove it if it contains data from an other data set"
	echo "Please type the option number:"
	select ync in "Yes" "No" "Cancel"; do
		case $ync in
			Yes ) rm -r data; break;;
			No ) break;;
			Cancel ) exit;;
		esac
	done
fi
#Preprocess input image data

mkdir -p data/left
mkdir -p data/right

ffmpeg -i $1 -vf "scale=768:ih*768/iw" data/left/%04d.png &
ffmpeg -i $2 -vf "scale=768:ih*768/iw" data/right/%04d.png &

wait
rm data/timestamps.txt
for filename in data/left/*.png; do
	base=$(basename $filename) 
	echo ${base%.*} >> data/timestamps.txt
done

#Run orb slam 2 to extract camera path
unamestr=`uname`
if [[ "$unamestr" != 'Darwin' ]]; then
	cd ORB_SLAM2

	./Examples/Stereo/stereo_saab Vocabulary/ORBvoc.txt ../$3 ../data/left ../data/right ../data/timestamps.txt

	mv CameraTrajectory.txt ../data
	mv KeyFrameTrajectory.txt ../data

	cd ..
fi

# rotate cameras to align to xz-plane
mv data/KeyFrameTrajectory.txt data/keyframes.bak
misc/build/rotcam data/keyframes.bak data/KeyFrameTrajectory.txt
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

iter=0

for filename in ../lrect/*.png; do

    echo Creating a disparity map for $filename

    if (( "$iter" > 1 )) ; then
    	wait
		iter=0
	fi
	base=$(basename $filename) 
 	#../../dispmap/disparitymap ../lrect/$base ../rrect/$base & # uncomment to use own disparity map
	../../spsstereo/build/spsstereo ../lrect/$base ../rrect/$base # && # comment to use own disparity map
	mv ${base%.*}_left_disparity.png $base &

	iter=$((iter+1))
done
# wait for last disparity maps
wait

cd ../../

#Create PCD files
mkdir -p data/pcd/final
mkdir -p data/pcd/cut
cd data/pcd

for filename in ../disp/*.png; do 
	base=$(basename $filename)
	../../3drecon/disp2cloud ../lrect/$base $filename ../../$3
done

cd ../../

#Merge all PCDs

cp pcdfile/emptyCloud.pcd data/pcd/final/finalCloud.pcd

misc/build/keyframe2pcd data/KeyFrameTrajectory.txt data/pcd/final/camerapos.pcd

while IFS='' read -r line || [[ -n "$line" ]]; do
    number=$(echo "$line" | awk '{print $1;}')
	trans=$(echo "$line"  | awk '{$1=""; print $0}')
	printf -v number "%04d" ${number%.*}
#	echo $number
	echo $trans
	pcdname=$number.pcd
	pcd_progs/build/cutting2 data/pcd/$pcdname data/pcd/final/camerapos.pcd $trans data/pcd/cut/$pcdname
done < "data/KeyFrameTrajectory.txt"

pcl_concatenate_points_pcd data/pcd/cut/*

mv output.pcd data/pcd/final/raw_rgb_cloud.pcd

for filename in data/pcd/cut/*.pcd; do

    echo Using mls to smooth $filename

    if (( "$iter" > 2 )) ; then
        wait
        iter=0
    fi
    pcl_mls_smoothing $filename $filename -radius 0.03 -use_polynomial_fit 1 &

    iter=$((iter+1))
done

wait

while IFS='' read -r line || [[ -n "$line" ]]; do
    number=$(echo "$line" | awk '{print $1;}')
    trans=$(echo "$line"  | awk '{$1=""; print $0}')
    printf -v number "%04d" ${number%.*}
    #	echo $number
    echo $trans
    pcdname=$number.pcd
	if [ -f data/pcd/cut/$pcdname ]
	then
		pcd_progs/build/merge data/pcd/cut/$pcdname data/pcd/final/finalCloud.pcd 0.03 $trans
	fi
done < "data/KeyFrameTrajectory.txt"

echo Cleaning pointcloud
pcd_progs/build/cleanup data/pcd/final/finalCloud.pcd 0.01 8 0.01
echo Done!

#Mesh final PCD

echo Generating mesh
pcl_poisson_reconstruction finalCloud_clean.pcd output.vtk
pcl_vtk2ply output.vtk mesh.ply
rm output.vtk
echo Done

#Map textures to the mesh
mesh/build/texturemapping mesh.ply data/KeyFrameTrajectory.txt stereo_cam.yml

#Open blender to calc volume
blender blender/emptyscene.blend --addons object_print3d_utils object_boolean_tools --python blender/startup.py

