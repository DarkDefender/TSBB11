#!/bin/bash 
if (( $# != 3 )); then
    echo "Illegal number of parameters"
	echo "Usage:"
	echo "$0 <left_img_seq>.avi <right_img_seq>.avi <orb_settings>.yml square_size board_width board_height"
	# square_size = 0.035 board_width = 8 board_height = 6
	exit 2
fi
echo Running stereo calibration with parameters:
echo $1
echo $2
echo $3
echo $4
echo $5

mkdir -p data/calib/left
mkdir -p data/calib/right

echo splitting avis to imgs...
 
ffmpeg -i $1 -r 0.5 data/calib/left/%d.png &
ffmpeg -i $2 -r 0.5 data/calib/right/%d.png &

wait

echo calibrating intrinsic parameters...

stereo-calibration/build/calibrate -i data/calib/left/%d.png -o data/calib/l_cam.yml -s $3 -w $4 -h $5 
# usage: ./calibrate -i input -o output -s square_size -w board_width -h board_height
stereo-calibration/build/calibrate -i data/calib/right/%d.png -o data/calib/r_cam.yml -s $3 -w $4 -h $5 
# usage: ./calibrate -i input -o output -s square_size -w board_width -h board_height


echo calibrating stereo parameters...
stereo-calibration/build/calibrate_stereo -l data/calib/left/%d.png -r data/calib/right/%d.png -u data/calib/l_cam.yml -v data/calib/r_cam.yml -o data/calib/stereo_cam.yml
# usage: calibrate_stereo -l left_seq -r right_seq -u left_calib -v right_calib -o output

echo adding ORB-SLAM settings...
misc/build/orbsetting data/calib/stereo_cam.yml $3 data/calib/orb_cam.yml

#rm stereo_cam.yml
#touch stereo_cam.yml

cp $3 stereo_cam.yml
echo "###############################" >> stereo_cam.yml
echo "# CAMERA SETTINGS FOR ORBSLAM \#" >> stereo_cam.yml
echo "###############################" >> stereo_cam.yml 
sed -i -e 's/%/#/g' data/calib/orb_cam.yml # replace yaml header with comment 
cat data/calib/orb_cam.yml >> stereo_cam.yml
echo "###############################" >> stereo_cam.yml
echo "# CAMERA CALIBRATION FOR RECT \#" >> stereo_cam.yml
echo "###############################" >> stereo_cam.yml  
sed -i -e 's/%/#/g' data/calib/stereo_cam.yml # replace yaml header with comment 
cat data/calib/stereo_cam.yml >> stereo_cam.yml
# replace underscore with dot
sed -i -e 's/_/./g' stereo_cam.yml

echo done!
