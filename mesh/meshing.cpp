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
#include <string>


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
#include <pcl/visualization/keyboard_event.h>


using namespace pcl;
using namespace std;

//Parameters
double searchRadius = 1.5;
double mu = 3;
unsigned maxNN = 400;
double maxSurfAng = M_PI/2;
double minAng = M_PI/36;
double maxAng = M_PI/2;
bool normalConsistency = true;

bool keyFlag = true;

//KeyboardEvent
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.keyDown () && keyFlag){
        keyFlag = false;
        cout << event.getKeySym() << endl;
        const char * key = event.getKeySym().c_str();
        switch ((int)key[0]){
            case (int)'w' :
                cout << "w was pressed => searchRadius increased" << endl;
                searchRadius = searchRadius + 0.1;
                cout << searchRadius << endl;
                break;
            case (int)'s' :
                cout << "s was pressed => searchRadius decreased" << endl;
                searchRadius = searchRadius - 0.1;
                cout << searchRadius << endl;
                break;
            case (int)'y' :
                cout << "y was pressed => mu increased" << endl;
                mu = mu + 0.1;
                cout << mu << endl;
                break;
            case (int)'h' :
                cout << "h was pressed => mu decreased" << endl;
                mu = mu - 0.1;
                cout << mu << endl;
                break;
            case (int)'t' :
                cout << "t was pressed => maxNN increased" << endl;
                maxNN = maxNN + 20;
                cout << maxNN << endl;
                break;
            case (int)'g' :
                cout << "g was pressed => maxNN decreased" << endl;
                maxNN = maxNN - 20;
                cout << maxNN << endl;
                break;
        }
    }
    if(event.keyUp()){
        keyFlag = true;
    }
}

int
main (int argc, char** argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPLYFile ("../pcdfile/punkthog.ply", cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    //* the data should be available in cloud
     
     //---------
     // GREEDY
     //---------
     
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
    gp3.setSearchRadius (searchRadius);
    
    // Set typical values for the parameters
    gp3.setMu (mu);
    gp3.setMaximumNearestNeighbors (maxNN);
    gp3.setMaximumSurfaceAngle(maxSurfAng); // 45 degrees
    gp3.setMinimumAngle(minAng); // 10 degrees
    gp3.setMaximumAngle(maxAng); // 120 degrees
    gp3.setNormalConsistency(normalConsistency);
    
    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    
    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(triangles,"meshes",0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
        if(!keyFlag){
            viewer->removePolygonMesh("meshes");
            gp3.setSearchRadius (searchRadius);
            gp3.setMu (mu);
            gp3.setMaximumNearestNeighbors (maxNN);
            gp3.setMaximumSurfaceAngle(maxSurfAng); // 45 degrees
            gp3.setMinimumAngle(minAng); // 10 degrees
            gp3.setMaximumAngle(maxAng); // 120 degrees
            gp3.setNormalConsistency(normalConsistency);
            
            // Get result
            gp3.setInputCloud (cloud_with_normals);
            gp3.setSearchMethod (tree2);
            gp3.reconstruct (triangles);
            
            // Additional vertex information
            std::vector<int> parts = gp3.getPartIDs();
            std::vector<int> states = gp3.getPointStates();
            viewer->addPolygonMesh(triangles,"meshes",0);
        }
    }
    pcl::io::saveOBJFile ("../objects/mesh.obj", triangles);
    
    // Finish
    return (0);
    
}