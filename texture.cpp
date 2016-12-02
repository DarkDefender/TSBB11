//
//  texture.cpp
//  
//
//  Created by Max Gefvert on 2016-11-25.
//
//

#include <stdio.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/PolygonMesh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/cloud_iterator.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/processing.h>
#include <pcl/features/integral_image_normal.h>
using namespace pcl;

int main (int argc, char **argv)
{
    // CREATE POINTCLOUDS AND POLYGONMESH
    PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB> ());
    PointCloud<PointXYZRGB>::Ptr ogCloud (new PointCloud<PointXYZRGB> ());
    PolygonMesh mesh;

    //INPUT
    io::loadPolygonFile (argv[1], mesh);
    io::loadPCDFile (argv[2], *ogCloud);

    //EXTRACT PC FROM MESH
	fromPCLPointCloud2(mesh.cloud, *cloud);

    PointCloud<PointXYZRGB>::iterator iter;


    for(iter = cloud->points.begin(); iter < cloud->points.end(); iter++) {

    	PointXYZRGB searchPoint = *iter;

        KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud (ogCloud);
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
      	std::vector<float> pointNKNSquaredDistance(K);


      	if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      	{
        	for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i) {
          		// std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
          		// << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
            //     << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
            //     << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;

                iter->r = ogCloud->points[ pointIdxNKNSearch[i] ].r;
                iter->g = ogCloud->points[ pointIdxNKNSearch[i] ].g;
                iter->b = ogCloud->points[ pointIdxNKNSearch[i] ].b;
            }
        }
    }




    // ---------------
    // Visualize
    // ---------------
	
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(cloud, 255, 0, 0);
    visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green_color(ogCloud, 0, 255, 0);

    // viewer->addPointCloud(cloud, red_color, "mesh");
    viewer->addPointCloud(cloud, "ogcloud");

    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
    
   