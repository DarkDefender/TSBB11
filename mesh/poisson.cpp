//
//  poisson.cpp
//
//
//  Created by Johan Lind on 2016-11-16.
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
    if (argc != 3)
    {
        PCL_ERROR ("Syntax: %s input.pcd radius(float)\n", argv[0]);
        return -1;
    }
    
    // CREATE POINTCLOUDS OF POINT, OF NORMALS AND OF POINT NORMALS
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
    PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
    PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal> ());
    
    //INPUT
    io::loadPCDFile (argv[1], *cloud);
    float searchRadius = atof(argv[2]);
    
    //MOVING LEAST SQUARES (SMOOTHING)
    MovingLeastSquares<PointXYZ, PointXYZ> mls;
    mls.setInputCloud (cloud);
    mls.setSearchRadius (searchRadius);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::NONE);
   // mls.setUpsamplingRadius (0.05);
   // mls.setUpsamplingStepSize (0.03);
    mls.process (*cloud_smoothed);
    
    // ESTIMATE NORMALS
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setNumberOfThreads (8);
    ne.setInputCloud (cloud_smoothed);
    ne.setRadiusSearch (searchRadius);
    Eigen::Vector4f centroid;
    compute3DCentroid (*cloud_smoothed, centroid);
    ne.setViewPoint (centroid[0], centroid[1] - 5, centroid[2]);
    
    ne.compute (*cloud_normals);
   
    /*
     for (size_t i = 0; i < cloud_normals->size (); ++i)
    {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
*/
    
    //CONCATENATE FIELDS
    concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_with_normals);
    
    //----------------
    //ALTERNATIVE 2
    //----------------
   /*
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);
    
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls2;
    
    mls2.setComputeNormals (true);
    
    // Set parameters
    mls2.setInputCloud (cloud);
    mls2.setPolynomialFit (true);
    mls2.setSearchMethod (tree);
    mls2.setSearchRadius (searchRadius);
    
    // Reconstruct
    mls2.process (*mls_points);
    
    */
    //-----------------------
    // POISSON
    //----------------------
    
    Poisson<PointNormal> poisson;
    poisson.setDepth (9);
    poisson.setInputCloud(cloud_with_normals);
    PolygonMesh mesh;
    poisson.reconstruct (mesh);

    // ---------------
    // Visualize
    // ---------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(mesh, "mesh");
    //viewer->addPointCloud(cloud,"pointCloud");
    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    io::saveOBJFile("../../objects/poissonMesh.obj",mesh);
    return 0;
}