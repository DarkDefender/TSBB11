//
//  normalCalc.cpp
//  
//
//  Created by Johan Lind on 2016-11-18.
//
//

#include <stdio.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/processing.h>
#include <pcl/io/obj_io.h>
#include <pcl/features/integral_image_normal.h>

using namespace pcl;
int
main (int argc, char **argv)
{
    /*if (argc != 4)
    {
        PCL_ERROR ("Syntax: %s newCloud.pcd oldCloud.pcd radius(float)\n", argv[0]);
        return -1;
    }*/
    
    // CREATE POINTCLOUDS OF POINT, OF NORMALS AND OF POINT NORMALS
    PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB> ());
    PointCloud<PointXYZRGBNormal>::Ptr oldCloud (new PointCloud<PointXYZRGBNormal> ());
    
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
    PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals (new PointCloud<PointXYZRGBNormal> ());
    
    //INPUT
    io::loadPCDFile (argv[1], *cloud);
    io::loadPCDFile (argv[2], *oldCloud);
    float searchRadius = atof(argv[3]);
    
    
    // ESTIMATE NORMALS
    NormalEstimationOMP<PointXYZRGB, Normal> ne;
    ne.setNumberOfThreads (8);
    ne.setInputCloud (cloud);
    ne.setRadiusSearch (searchRadius);
    ne.setViewPoint (atof(argv[4]), atof(argv[5]), atof(argv[6]));
    ne.compute (*cloud_normals);
    
    //CONCATENATE FIELDS
    concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);
    
    std::cout<<"size of oldcloud: "<< oldCloud->size()<<std::endl;
    
    //merge clouds
    *oldCloud += *cloud_with_normals;
    
    std::cout<<"size of final cloud: "<< oldCloud->size()<<std::endl;
	pcl::io::savePCDFile (argv[2], *oldCloud, true); 
	// ---------------
    // Visualize
    // ---------------
  /*  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    viewer->addPointCloud(oldCloud,"pointCloud");
    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
   */
    return 0;
}
