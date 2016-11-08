#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/pcl_base.h>
#include <pcl/io/obj_io.h>
#include "popt.h"

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/keyboard_event.h>
/* Parameters to adjust are 
 */
float filterLimit = 4.0;
float distanceThreshold = 100;
float alphaValue = 0.15;
static char *filename;

static struct poptOption optionsTable[] = {
    {"filterLimit", 'f', POPT_ARG_FLOAT, &filterLimit, 0, "the limits of the filter",NULL},
    {"pointCloud", 'p', POPT_ARG_STRING, &filename, 0, "name of point cloud",NULL},
    {"distanceThreshold", 'd', POPT_ARG_FLOAT, &distanceThreshold, 0, "distance threshold",NULL},
    {"alphaValue", 'a', POPT_ARG_FLOAT, &alphaValue, 0, "alpha value",NULL},
    
    POPT_AUTOALIAS
    POPT_AUTOHELP
    POPT_TABLEEND
};

int main (int argc, char** argv)
{
    poptContext context = poptGetContext(
    (char*) "popt1", argc,(const char **) argv,
                                         (const struct poptOption* ) &optionsTable,
                                         0);
    int option = poptGetNextOpt(context);
                                         
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
    cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
    cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    
    
    reader.read(filename, *cloud);
   
    
    
    
    
    // Build a filter to remove spurious NaNs
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, filterLimit);   //(0,3.0) for triang
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: "
    << cloud_filtered->points.size () << " data points." << std::endl;
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold); //0.01
    
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
    << inliers->indices.size () << " inliers." << std::endl;
    
    pcl::PCDWriter writer;
    
    pcl::PolygonMesh mesh;
    std::string mesh_id = "mesh";
    std::vector< pcl::Vertices > meshVect;
    
    pcl::ConcaveHull<pcl::PointXYZ> hullObj;
    hullObj.setDimension(3);
    hullObj.setInputCloud (cloud_filtered);
    hullObj.setAlpha(alphaValue);
    hullObj.reconstruct(mesh);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);           // black background
    //viewer->addPointCloud(cloud_filtered,"meshes",0); //add original point cloud
    //viewer->addCoordinateSystem (1.0);        //show coordinate axis
    viewer->initCameraParameters ();
    viewer->addPolygonMesh (mesh, mesh_id);
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
     pcl::io::saveOBJFile ("../objects/mesh.obj", mesh);
    
    
    return (0);
}