#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>

// #include "/home/max/ws_livox/src/livox_ros_driver2/src/include/ros1_headers.h"

// using PointCloud2 = sensor_msgs::PointCloud2;


void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->header.frame_id.c_str());
    ROS_INFO("pointcloud2 size w <%d> h <%d>",msg->width,msg->height);
    std::cout<< "fields[] " <<msg->fields.size()<< std::endl;
    for (size_t i = 0; i < msg->fields.size(); ++i)
    {
        std::cout <<msg->fields[i].name<<" ";
    }
    std::cout<<std::endl;
}

int main(int argc, char** argv)
{
    std::string pointcloud2_topic = "/livox/lidar";
    if (argc > 1){
        pointcloud2_topic = std::string(argv[1]);
    }
    ROS_INFO("Point Cloud2 Topic <%s>",pointcloud2_topic.c_str());

    ros::init(argc, argv, "sub_pc");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(pointcloud2_topic, 1000, pcCallback);

    ros::spin();

    return 0;
}