#ifndef SLAM_HPP
#define SLAM_HPP
#include "includes.ihh"

#define M_PIf 3.14159265358979f

using namespace noos::cloud;
/**
 * @brief read the laser data
 * @class slam
 */
class slam
{
public:
    /// @brief constructor
    slam();

    /// @brief read laser data
    void read_laser(const sensor_msgs::LaserScan::ConstPtr & scan);

    /// @brief the data is sent to the platform
    void process_data(noos::object::laser & obs);

private:
    //Callback
    void callback(noos::object::pose<float> pose3d);

    //Callable object
    callable<icp_slam, true> callab_;
    //Count time between calls
    double t_begin__;

};

#endif
