CDIO Project: 3D Mapping of Outdoor Scenes Using Stereo Imagery
========

This project provides a pipeline to generate a mesh of simple real life objects
that has been filmed by a stereo camera. The goal is to then use the generated
mesh to create a volume approximation of the object (using blender) 

## Dependencies

- CMake
- GCC (the ORB-SLAM2 version used in this repo does not compile with clang)
- [OpenCV](http://opencv.org)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [BLAS](http://www.netlib.org/blas/)
- [LAPACK](http://www.netlib.org/lapack/)
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- [PCL](http://pointclouds.org/)
- [Png++](http://www.nongnu.org/pngpp/)
- [Blender](http://www.blender.org)

## Build instructions

Make sure you have all the dependencies listed above and that you have all the git submodules downloaded.
(Clone this repo with the "--recursive" flag)

Then just run the build.sh script to build everything

## Running the calibration script

In order for the pipeline to work, the camera parameters need to be known. To
calibrate, record a short sequence of a checkered board. The board needs to be
flat and the size of the squared need to be known. Rotate the board in
different orientation. Try to move around the board so that it has traversed
all corners of the cameras field of view and that it is visible for both
cameras. 

The recorded calibration sequence should be saved as two video files. One file
for the left camera and one for the right.

To perform the actual calibration, simply run the calibration script with the
left and right sequence, the size of the squares in meters, the width and
height of the board (meaning the number of midpoints). For example:

`./calib_stereo.sh left.avi right.avi 0.035 8 6`

## The run script

When building and calibration is done, run the binary "run.sh" with the path to
the left and right camera files and the calibration file, e.g:

`./run.sh left.avi right.avi data/calib/stereo_cam.yml`

At the end blender will open and you can use the bundled blender addons "Bool
Tool" and "3D print toolbox" to cut out and calcutate the volume of the
generated mesh.

## Screenshots

A frame from our "box1" data set.
![Frame1](https://github.com/DarkDefender/TSBB11/raw/master/doc_res/0001.png)
And the generated mesh:
![Mesh](https://github.com/DarkDefender/TSBB11/raw/master/doc_res/box1.gif)

#### Mesh accuracy
On the "box1" data set, the accuracy seems to be quite good. **However**, more
test data is needed to determine is the mesh generation is robust enough.

The "real life" measurements were done using a folding ruler. The mesh
measurements were done in blender. For an explanation of the different length
labels, see this image: 
[Box1 labels](https://github.com/DarkDefender/TSBB11/raw/master/doc_res/box_dim.png)

|     | Real life | Mesh |  
| --- | --- | --- |
| Height [cm] | 50 | 49.7 |
| Width long [cm] | 84 | 83.9 |
| Width short [cm] | 65 | 64.5 |
| Lenght long [cm] | 80 | 80.3 |
| Lenght short [cm] | 75 | 75.4 |
| Volume [m^3] | 0.3313 | 0.3339 |

## Documentation PDFs

[User Guide](https://github.com/DarkDefender/TSBB11/raw/master/doc_res/CDIO_user_guide.pdf)
