//
//  cutting.cpp
//
//
//  Created by Johan Lind on 2016-11-03.
//
//
 
#include "cutting.hpp"

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



int
main (int argc, char** argv)
{

    //Create point cloud ptrs
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_poses (new pcl::PointCloud<pcl::PointXYZ>);
    
    // load the point cloud and poses point cloud
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("../pcdfile/pc000100.pcd", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    
    pcl::PCLPointCloud2 poses;
    pcl::io::loadPCDFile ("../pcdfile/CameraPositions.pcd", poses);
    pcl::fromPCLPointCloud2 (poses, *cloud_poses);
    
    //-----------------------------
    // Crop Hull
    //------------------------------
    std::vector<pcl::Vertices> vertices;
    pcl::Vertices vt;
    for (int i = 0; i<cloud_poses->size(); i++){
        vt.vertices.push_back(i);
    }
    vertices.push_back(vt);
    
    pcl::CropHull<pcl::PointXYZ> cropHull;
    // build the filter
    cropHull.setDim(2);
    cropHull.setHullIndices(vertices);
    cropHull.setHullCloud(cloud_poses);
    cropHull.setInputCloud(cloud);
    cropHull.setCropOutside(true);
    
    // apply filter
    cropHull.filter(*cloud_filtered);
    
    /*
    //--------------------------------
    // FILTERING
    //--------------------------------
    
    // build the conditions
    // points within radius from centroid are kept (in x,y,z field)
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, centroidXYZ.x-radiusAllowed)));
    
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, centroidXYZ.x+radiusAllowed)));
    
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, centroidXYZ.y-radiusAllowed)));
    
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, centroidXYZ.y+radiusAllowed)));
    
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, centroidXYZ.z-radiusAllowed)));
    
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, centroidXYZ.z+radiusAllowed)));
    
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(false);
    // apply filter
    condrem.filter (*cloud_filtered);
     */
    
    // ----------------
    // Remove Noise
    // ----------------
    
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius (6);
    // apply filter
    //outrem.filter (*cloud_filtered);
    
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);
    
    std::cout<<"size before filt: "<< cloud->size()<<std::endl;
    std::cout<<"size after filt: "<< cloud_filtered->size()<<std::endl;

    // ----------------
    // Rotate ORB SLAM PC to this coord sys
    // ----------------

    // Viewpoints exported from ORB-SLAM2
    // 100: 0.000000 -0.194615707 0.062869363 0.166686013 x= -0.093350478 y= -0.075912602 z= 0.017663542 w= 0.992577910
    // 200: 0.000000 -1.636660933 0.150365740 0.651330590 -0.050624952 0.245877609 0.018973112 0.967791975
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr orb(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr orbZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr orbTrans(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 orbLoad;
    pcl::io::loadPCDFile ("../pcdfile/mapPoints.pcd", orbLoad);
    pcl::fromPCLPointCloud2 (orbLoad, *orb);

    Eigen::Quaternionf Q(0.992577910, -0.093350478, -0.075912602, 0.017663542);
    Eigen::Matrix<float, 3, 1> translation;
    translation << -0.194615707, 0.062869363, 0.166686013;
    Eigen::Affine3f rotZ = Eigen::Affine3f::Identity();
    rotZ.rotate(Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitZ()));

    // Rotates pi around z...
    pcl::transformPointCloud(*orb, *orbZ, rotZ);
    // Transforms
    pcl::transformPointCloud(*orbZ, *orbTrans, translation, Q);
 
    // ---------------
    // Visualize
    // ---------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_poses, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> greenColor(cloud_poses, 0, 255, 0);

    viewer->addPointCloud(cloud_poses,single_color, "cameraPath");
    viewer->addPointCloud(cloud_filtered,"filtCloud");
    viewer->addPointCloud(orbTrans, greenColor, "orbTrans");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    pcl::io::savePLYFile ("../pcdfile/point_filt.ply", *cloud_filtered);

    return (0);
}