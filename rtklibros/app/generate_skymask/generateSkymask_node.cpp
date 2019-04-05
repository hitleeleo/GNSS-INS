#include "generateSkymask.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "generateskymask_node");
    std::cout<<"generateskymask_node......"<<std::endl;

    // get parameters
    std::string directory,saved_skymask_dir;
    int skymask_grid_resolution = 2;

    bool mannually_set_ref_llh = false;
    double ref_lat = 0.0;
    double ref_lon = 0.0;

    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("directory", directory, "/home/wenws/amsipolyu/src/rtklibros/app/generate_skymask/data/demo_HK_TST_fix.kml"); 
    nh_private.param<int>("skymask_grid_resolution", skymask_grid_resolution, 2); 
    nh_private.param<std::string>("saved_skymask_dir", saved_skymask_dir, "/home/wenws/amsipolyu/src/rtklibros/app/generate_skymask/skymask/"); 
    
    nh_private.param<bool>("mannually_set_ref_llh", mannually_set_ref_llh, false); 
    nh_private.param<double>("ref_lat", ref_lat, 0.0); 
    nh_private.param<double>("ref_lon", ref_lon, 0.0); 

    // printf("obss.n = %d\n", obss.n);
    generateSkymask generateSkymask_(directory,saved_skymask_dir,mannually_set_ref_llh, ref_lat,ref_lon);
    ros::spinOnce();
    while (ros::ok()) {

    }
    return 0;
}
