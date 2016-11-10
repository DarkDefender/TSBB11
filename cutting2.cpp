//
//  cutting2.cpp
//  
//
//  Created by Johan Lind on 2016-11-10.
//

#include <cstdlib>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_hull.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

using namespace std;

//-----------------------------
// Crop Hull
//-----------------------------

void cropPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropCloud,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPoses){

std::vector<pcl::Vertices> vertices;
pcl::Vertices vt;
for (int i = 0; i< cloudPoses->size(); i++){
    vt.vertices.push_back(i);
}
vertices.push_back(vt);

pcl::CropHull<pcl::PointXYZRGB> cropHull;
// build the filter
cropHull.setDim(2);
cropHull.setHullIndices(vertices);
cropHull.setHullCloud(cloudPoses);
cropHull.setInputCloud(inputCloud);
cropHull.setCropOutside(true);

// apply filter
cropHull.filter(*cropCloud);
    
}

//------------------------------
// Rotate and translate
//------------------------------
void translatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud,
                         Eigen::Quaternionf Q,
                         Eigen::Matrix<float, 3, 1> translation){
    // Transforms
    pcl::transformPointCloud(*inputCloud, *outputCloud, translation, Q);
}

// ----------------
// Filtering with VoxelGrid
// ----------------

void voxelGridFiltering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud,
                        float leafSize){

// Create the filtering object
pcl::VoxelGrid<pcl::PointXYZRGB> sor;
sor.setInputCloud (inputCloud);
sor.setLeafSize (leafSize, leafSize, leafSize);
sor.filter (*outputCloud);
}

// ----------------
// Radius Outlier Removal
// ----------------
void radiusFiltering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud,
                     float radius,
                     int minNeighbors){
    
pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
outrem.setInputCloud(inputCloud);
outrem.setRadiusSearch(radius);
outrem.setMinNeighborsInRadius (minNeighbors);
outrem.filter (*outputCloud);
}

int
main (int argc, char** argv)
{
  /*  if (argc != 3)
    {
        std::cerr << "need to specify: newCloud, oldCloud, camPoses(cloud), Q, t" << std::endl;
        exit(0);
    }*/
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origoCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB origo;
    origo.x = 0;
    origo.y = 0;
    origo.z = 0;
    origoCloud->push_back(origo);
    
    pcl::io::savePCDFile ("../pcdfile/emptyCloud.pcd", *origoCloud);
     
     */
    
    //Create point cloud ptrs
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr oldCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPoses (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGB> (argv[1], *newCloud);
    reader.read<pcl::PointXYZRGB> (argv[2], *oldCloud);
    reader.read<pcl::PointXYZRGB> (argv[3], *cloudPoses);
    
    //TEMPORARY
  //  Eigen::Quaternionf Q2(0.967791975, -0.050624952, 0.245877609, 0.018973112);
  //  Eigen::Matrix<float, 3, 1> trans2;
  //  trans2 << -1.636660933, 0.150365740, 0.651330590;
    
    Eigen::Quaternionf Q2(atof(argv[10]), atof(argv[7]),atof(argv[8]),atof(argv[9]));
    Eigen::Matrix<float, 3, 1> trans2;
    trans2 << atof(argv[4]), atof(argv[5]),atof(argv[6]);

    //Crop
 //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    //cropPoints(oldCloud,finalCloud,cloudPoses); //input,output,camerapath
    
    //Translate
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    translatePointCloud(newCloud,transCloud,Q2,trans2);
    
    //Crop
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropAndTransCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    cropPoints(transCloud,cropAndTransCloud,cloudPoses);
    
    //VoxelGrid filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    voxelGridFiltering(cropAndTransCloud,filteredCloud,0.005f);
    
    std::cout<<"size before crop: "<< newCloud->size()<<std::endl;
    std::cout<<"size after crop and filt: "<< filteredCloud->size()<<std::endl;
    std::cout<<"size of old cloud: "<< oldCloud->size()<<std::endl;
    
    //merge clouds
    *oldCloud += *filteredCloud;
    
    std::cout<<"size of final cloud: "<< oldCloud->size()<<std::endl;
    
    // ---------------
    // Visualize
    // ---------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    
   // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> redColor(cloudPoses, 255, 0, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> greenColor(oldCloud, 0, 255, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cyanColor(cropCloud, 0, 255, 255);
    
    
    viewer->addPointCloud(cloudPoses, "cameraPath");
    viewer->addPointCloud(oldCloud, "oldCloud");
 //   viewer->addPointCloud(cropCloud, cyanColor, "newCropedCloud");
    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    pcl::io::savePCDFile ("../pcdfile/finalCloud.pcd", *oldCloud);
    return (0);
}