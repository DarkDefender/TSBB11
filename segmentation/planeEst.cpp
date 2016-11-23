//
//  planeEst.cpp
//  
//
//  Created by Johan Lind on 2016-11-16.
//
//

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/keyboard_event.h>

int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGBNormal> (argv[1], *cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.5);
    
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    std::cout << coefficients->values[0] << std::endl;
    std::cout << coefficients->values[1] << std::endl;
    std::cout << coefficients->values[2] << std::endl;
    std::cout << coefficients->values[3] << std::endl;
    
    
    
    
 /*   pcl::ProjectInliers<pcl::PointXYZRGBNormal> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
  */
    
    // ---------------
    // Visualize
    // ---------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud,"cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> single_color(cloud_projected, 255, 0, 0);
 //   viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud_projected,single_color, "filtCloud");
    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
    return (0);
}