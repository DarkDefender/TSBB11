#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/io/obj_io.h>
#include "popt.h"
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

/* Parameters to adjust are 
 */
float alphaValue = 0.15;
static char *filename;

static struct poptOption optionsTable[] = {                     // input arguments
    {"pointCloud", 'p', POPT_ARG_STRING, &filename, 0, "name of point cloud",NULL},
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
                                         
    //Read cloud from input file location
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
    cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>),
    cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    
    reader.read(filename, *cloud);
    
    // Create an empty mesh
    pcl::PolygonMesh mesh;
    std::string mesh_id = "mesh";
  
    // Construct a polygon mesh from the point cloud
    pcl::ConcaveHull<pcl::PointXYZRGB> hullObj;
    hullObj.setDimension(3);
    hullObj.setKeepInformation(true);
    hullObj.setInputCloud (cloud);
    hullObj.setAlpha(alphaValue);
    hullObj.reconstruct(mesh);

    // Create visualization object
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (1, 0, 0);           // black background
    //viewer->addPointCloud(cloud_filtered,"meshes",0); //add original point cloud
    //viewer->addCoordinateSystem (1.0);        //show coordinate axis
    viewer->initCameraParameters ();
    viewer->addPolygonMesh (mesh, mesh_id);
    
    // Visualize polygon mesh
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    // Save polygon mesh to file
     pcl::io::saveOBJFile ("../objects/mesh.obj", mesh);
    
    return (0);
}