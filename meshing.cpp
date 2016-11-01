//
//  
//
//  Created by Johan Lind on 2016-10-03.
//
//

#include "meshing.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <boost/thread/thread.hpp>



#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>


#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//using namespace cv;
//using namespace std;

/*void meshing(pcl::PointCloud<pcl::PointXYZ> point_cloud){
    
}
*/

using namespace pcl;

int
main (int argc, char** argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPLYFile ("../pcdfile/punkthog.ply", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    //* the data should be available in cloud
    
 /*   MovingLeastSquares<PointXYZ, PointXYZ> mls;
    mls.setInputCloud (cloud);
    mls.setSearchRadius (0.1);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.005);
    mls.setUpsamplingStepSize (0.003);
    PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
    mls.process (*cloud_smoothed);
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setNumberOfThreads (8);
    ne.setInputCloud (cloud_smoothed);
    ne.setRadiusSearch (0.1);
    Eigen::Vector4f centroid;
    compute3DCentroid (*cloud_smoothed, centroid);
    ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
    ne.compute (*cloud_normals);
    for (size_t i = 0; i < cloud_normals->size (); ++i)
    {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
    concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
    
    Poisson<PointNormal> poisson;
    poisson.setDepth (9);
    poisson.setInputCloud (cloud_smoothed_normals);
    PolygonMesh mesh;
    poisson.reconstruct (mesh);
    
    io::savePCDFile ("../pcdfile/cloudSmooth.pcd", *cloud_smoothed);
    io::saveOBJFile ("../objects/meshPOI.obj", mesh);

    
    //---------
    // GREEDY
    //---------
    
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.9);
    
    // Set typical values for the parameters
    gp3.setMu (2.0);
    gp3.setMaximumNearestNeighbors (300);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    
    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    
    pcl::io::saveOBJFile ("../objects/mesh.obj", triangles);
    
    // Finish
    return (0);
    
      */
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures
    
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);
    
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (2);
    
    // Set typical values for the parameters
    gp3.setMu (3);
    gp3.setMaximumNearestNeighbors (350);
    gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
    gp3.setMinimumAngle(M_PI/36); // 10 degrees
    gp3.setMaximumAngle(M_PI/2); // 120 degrees
    gp3.setNormalConsistency(true);
    
    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    
    pcl::io::saveOBJFile ("../objects/mesh.obj", triangles);
    
    // Finish
    return (0);
    
}