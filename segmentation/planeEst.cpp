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

#include <pcl/common/transforms.h>


int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudTrans(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
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
    seg.setDistanceThreshold (0.05);
    
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    //-------------
    // Flip point cloud around plane
    // ------------
    Eigen::Vector3f normal(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
    
    if (normal(1) > 0)
    {
        normal(0) *= -1;
        normal(1) *= -1;
        normal(2) *= -1;
    }
    
    Eigen::Vector3f xy_plane_normal_vector(0,1,0);
    Eigen::Vector3f rotation_vector = xy_plane_normal_vector.cross(normal);
    float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(normal));
    
    std::cout << "Rotation vector: "<< std::endl;
    std::cout << rotation_vector(0) << std::endl;
    std::cout << rotation_vector(1) << std::endl;
    std::cout << rotation_vector(2) << std::endl;
    std::cout << "Rotation angle: "<< std::endl;
    std::cout << theta << std::endl;
    
    Eigen::AngleAxisf rot(theta,rotation_vector/rotation_vector.norm());
    Eigen::Quaternionf quat(rot);
    Eigen::Matrix<float, 3, 1> trans;
    trans << 0, 0, 0;
    
    pcl::transformPointCloudWithNormals(*cloud, *cloudTrans,trans, quat);
    
    // The Plane
    pcl::ModelCoefficients plane_coeff;
    plane_coeff.values.resize (4);
    plane_coeff.values[0] = coefficients->values[0];
    plane_coeff.values[1] = coefficients->values[1];
    plane_coeff.values[2] = coefficients->values[2];
    plane_coeff.values[3] = coefficients->values[3];
    
    // ---------------
    // Visualize
    // ---------------
    /*
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> single_color(cloudTrans, 255, 0, 0);
    viewer->addPlane(plane_coeff,"plane");
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloudTrans,single_color,"cloud");
    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    */
    
    pcl::io::savePCDFile (argv[1], *cloudTrans, true);
    
    return (0);
}