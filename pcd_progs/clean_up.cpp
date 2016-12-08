//
//  cleanUp.cpp
//  
//
//  Created by Johan Lind on 2016-11-10.
//
//

#include <stdio.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/shadowpoints.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// ----------------
// Filtering with VoxelGrid
// ----------------

void voxelGridFiltering(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                        float leafSize){
    
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud (inputCloud);
    sor.setLeafSize (leafSize, leafSize, leafSize);
    sor.filter (*outputCloud);
}

// ----------------
// Radius Outlier Removal
// ----------------
void radiusFiltering(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                     float radius,
                     int minNeighbors){
    
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
    outrem.setInputCloud(inputCloud);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius (minNeighbors);
    outrem.filter (*outputCloud);
}

// ----------------
// Shadow filtering
// ----------------
void shadowFiltering(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr inputCloud,
                     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud,
                     float threshold){
    
    pcl::ShadowPoints<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal> shadRem(true);
    
    shadRem.setNormals(inputCloud);
    shadRem.setInputCloud(inputCloud);
    shadRem.setThreshold(threshold);
    shadRem.filter(*outputCloud);
}

int
main (int argc, char** argv)
{
    if (argc != 5)
    {
        std::cerr << "need to specify: " << std::endl;
        std::cerr << "pointCloud (.pcd)" << std::endl;
        std::cerr << "radius (float)" << std::endl;
        std::cerr << "minNeighbors (int)" << std::endl;
        std::cerr << "leafSize (float)" << std::endl;
    //    std::cerr << "shadFiltThresh (float)" << std::endl;
        exit(0);
    }
    
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudRadiusFilt (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudVoxelFilt (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudShadowFilt (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGBNormal> (argv[1], *cloud);
    float radius = atof(argv[2]);
    int minNeighbors = atoi(argv[3]);
    float leafSize = atof(argv[4]);
  //  float threshold = atof(argv[5]);
    
   /* //shadowpoint filtering
    shadowFiltering(cloud,cloudShadowFilt,threshold);
    
    std::cout << "cloud size: " << cloud->size() << std::endl;
    std::cout << "cloud after shadow filt: " << cloudShadowFilt->size() << std::endl;
    
    */
    
    //Radius filtering
    radiusFiltering(cloud,cloudRadiusFilt,radius, minNeighbors);
    
    std::cout << "cloud after radi: " << cloudRadiusFilt->size() << std::endl;
    
    //Voxel Filtering
    voxelGridFiltering(cloudRadiusFilt,cloudVoxelFilt,leafSize);
    
    std::cout << "cloud after radi and voxel: " << cloudVoxelFilt->size() << std::endl;
    
    // ---------------
    // Visualize
    // ---------------
	/*
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloudVoxelFilt, "filtCloud");
    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    */
    pcl::io::savePCDFile ("finalCloud_clean.pcd", *cloudVoxelFilt, true);
    return (0);
}
