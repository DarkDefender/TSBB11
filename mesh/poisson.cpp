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

using namespace pcl;
int
main (int argc, char **argv)
{
    if (argc != 2)
    {
        PCL_ERROR ("Syntax: %s input.pcd output.ply\n", argv[0]);
        return -1;
    }
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
    PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());

    
    io::loadPCDFile (argv[1], *cloud);
    
  /*  MovingLeastSquares<PointXYZ, PointXYZ> mls;
    mls.setInputCloud (cloud);
    mls.setSearchRadius (0.1);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.05);
    mls.setUpsamplingStepSize (0.03);
    mls.process (*cloud_smoothed);
    
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setNumberOfThreads (8);
    ne.setInputCloud (cloud_smoothed);
    ne.setRadiusSearch (0.01);
    Eigen::Vector4f centroid;
    compute3DCentroid (*cloud_smoothed, centroid);
    ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

    ne.compute (*cloud_normals);
    for (size_t i = 0; i < cloud_normals->size (); ++i)
    {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    PointCloud<PointNormal>::Ptr cloud_with_normals (new PointCloud<PointNormal> ());
    concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_with_normals);
    
   */
    
    //--------------------------
    
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);
    
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    
    mls.setComputeNormals (true);
    
    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);
    
    // Reconstruct
    mls.process (*mls_points);
    
    //-----------------------
    
    Poisson<PointNormal> poisson;
    poisson.setDepth (9);
    poisson.setInputCloud(mls_points);
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
    return 0;
}