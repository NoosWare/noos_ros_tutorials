#include "slam.hpp"

slam::slam() 
: callab_( std::bind(&slam::callback, this, std::placeholders::_1),
           default_node,
           "example_map", "icp.ini", noos::object::laser()),
  t_begin__(ros::Time::now().toSec())
{}

void slam::read_laser(const sensor_msgs::LaserScan::ConstPtr & scan)
{
    //
    //The frequency of the laser is higher than the processing time in 
    //the platform (40-50 ms). So the measures are going to be sent every
    //~100 ms.
    //
    if (ros::Time::now().toSec() - t_begin__ > 0.1) {
        assert(scan);
        noos::object::laser obs;
        if (scan) { 
            // 
            // All the parameters of laser MUST to be filled
            // For more information @see noos::object::laser
            //
            int count = scan->scan_time / scan->time_increment;
            auto now = std::chrono::system_clock::now();
            obs.timestamp = now.time_since_epoch().count();
            obs.ranges.resize(count);
            obs.intensities.resize(count);
            obs.right_to_left = false;
            obs.aperture = 2 * M_PIf;
            obs.max_range = 6.0;
            obs.std_error = 0.010f;
            obs.pose3d = noos::object::pose<float>();

            for (int i = 0; i < count; i++) {
                obs.ranges[i] = scan->ranges[i];
                obs.intensities[i] = (int)scan->intensities[i];
            }
            //
            //Now the laser object is complete, its data can be 
            //sent to the platform.
            process_data(obs);
        }
        else {
            std::cout << "No laser data" << std::endl;
        }
    }
}

void slam::process_data(noos::object::laser & obs)
{
    //
    //The object of the class callable is updated with the new laser data.
    //
    callab_.object = noos::cloud::icp_slam("example_map", "icp.ini", obs); //map, config, laser
    callab_.send();
}

void slam::callback(noos::object::pose<float> pose3d)
{
    //
    //The position of the robot in the map is showed
    //
    std::cout << pose3d;
    //
    //The time is reset to wait another 100 ms for the next call
    //
    t_begin__ = ros::Time::now().toSec();
}
