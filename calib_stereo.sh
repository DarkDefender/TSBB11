#!/bin/bash 
if (( $# != 2 )); then
    echo "Illegal number of parameters"
	echo "Usage:"
	echo "$0 <left_img_seq>.avi <right_img_seq>.avi"
	exit 2
fi
echo Running stereo calibration with parameters:
echo $1
echo $2

mkdir -p data/calib/left
mkdir -p data/calib/right

echo splitting avis to imgs...
 
ffmpeg -i $1 -r 1 data/calib/left/%d.png &
ffmpeg -i $2 -r 1 data/calib/right/%d.png &

wait

echo calibrating intrinsic parameters...

stereo-calibration/build/calibrate -i data/calib/left/%d.png -o data/calib/l_cam.yml -s 0.035 -w 8 -h 6 
# usage: ./calibrate -i input -o output -s square_size -w board_width -h board_height
stereo-calibration/build/calibrate -i data/calib/right/%d.png -o data/calib/r_cam.yml -s 0.035 -w 8 -h 6 
# usage: ./calibrate -i input -o output -s square_size -w board_width -h board_height

echo calibrating stereo parameters...
stereo-calibration/build/calibrate_stereo -l data/calib/left/%d.png -r data/calib/right/%d.png -u data/calib/l_cam.yml -v data/calib/r_cam.yml -o data/calib/stereo_cam.yml
# usage: calibrate_stereo -l left_seq -r right_seq -u left_calib -v right_calib -o output

echo adding ORB-SLAM parameters
frametxttopcd/build/orbsetting data/calib/stereo_cam.yml ORB_SLAM2/Examples/Stereo/ORB_SETTINGS.yml stereo_cam.yml

# replace underscore with dot
sed -i -e 's/_/./g' stereo_cam.yml

echo done!
