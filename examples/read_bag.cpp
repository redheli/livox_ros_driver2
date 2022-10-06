#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>

// #include "/home/max/ws_livox/src/livox_ros_driver2/src/include/ros1_headers.h"

// using PointCloud2 = sensor_msgs::PointCloud2;


rosbag::Bag bag;

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Syntax is: " << argv[0] << " <file_in.bag>" << std::endl;
        std::cerr << "Example: " << argv[0] << " data.bag" << std::endl;
        return (-1);
    }

    auto bag_file = std::string(argv[1]);
    ROS_INFO("Open bag file <%s>",bag_file.c_str());

    bag.open(bag_file);  // BagMode is Read by default

    int count=0;
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        ROS_INFO("message <%d> topic <%s>",count++,m.getTopic().c_str());
        if ( std::strcmp( m.getTopic().c_str(), "/livox/lidar") == 0)
        {
            sensor_msgs::PointCloud2::Ptr pc = m.instantiate<sensor_msgs::PointCloud2>();
            ROS_INFO("pointcloud2 size w <%d> h <%d>",pc->width,pc->height);
        }
    }

    bag.close();

    return 0;
}