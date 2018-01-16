#include "includes.ihh"
#include "slam.hpp"

int main(int argc, char **argv)
{
	using namespace noos::cloud;
    /* First send the config file */

    auto config = noos::object::config_file("data/icp.ini");

    auto request  = upload_slam_config_file(config, "icp.ini", slam_type::icp);
    auto config_callback = [](bool success) {
        std::cout << "Success uploading the config file: " 
			      << std::boolalpha << success << std::endl;
    };
    callable<upload_slam_config_file> config_callable(config_callback, 
                                                      default_node, 
                                                      config, 
                                                      "icp.ini", 
                                                      slam_type::icp);
    config_callable.send();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << " ================== " << std::endl;

	/* SLAM*/
	ros::init(argc, argv, "icp_slam");
	ros::NodeHandle n;

	ros::Rate loop_rate(1);
	slam slam_obj;

	// "/scan" is for RPLIDAR messages
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &slam::read_laser, &slam_obj);
	ros::spin();
	loop_rate.sleep(); 

    return 0;
}
