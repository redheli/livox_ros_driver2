#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>

// #include <opencv/cv.h>

#include <pcl/point_cloud.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

// #include "/home/max/ws_livox/src/livox_ros_driver2/src/include/ros1_headers.h"

// using PointCloud2 = sensor_msgs::PointCloud2;


Eigen::Matrix3d extRot;
Eigen::Matrix3d extRPY;
Eigen::Quaterniond extQRPY;


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

sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
{
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }

    return imu_out;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

    // std::lock_guard<std::mutex> lock1(imuLock);
    // imuQueue.push_back(thisImu);

    // debug IMU data
    std::cout << std::setprecision(6);
    std::cout << "IMU acc: " << std::endl;
    std::cout << "x: " << thisImu.linear_acceleration.x << std::endl;
    std::cout <<"y: " << thisImu.linear_acceleration.y << std::endl;
    std::cout <<"z: " << thisImu.linear_acceleration.z << std::endl;

    std::cout << "IMU gyro: " << std::endl;
    std::cout << "x: " << thisImu.angular_velocity.x << std::endl;
    std::cout <<"y: " << thisImu.angular_velocity.y << std::endl;
    std::cout <<"z: " << thisImu.angular_velocity.z << std::endl;
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImu.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    std::cout << "IMU roll pitch yaw: " << std::endl;
    std::cout << "roll: " << imuRoll << std::endl;
    std::cout <<"pitch: " << imuPitch << std::endl;
    std::cout <<"yaw: " << imuYaw << std::endl;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_pc");

// lio sam imu extrinsic param
    // extrinsicRot: [-1, 0, 0,
    //               0, 1, 0,
    //               0, 0, -1]
    // extrinsicRPY: [0, -1, 0,
    //                 1, 0, 0,
    //                 0, 0, 1]

    std::vector<double> extRotV{-1,0,0, 0,1,0, 0,0,-1};
    std::vector<double> extRPYV{0,-1,0, 1,0,0, 0,0,1};
    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
    extQRPY = Eigen::Quaterniond(extRPY).inverse();


    ros::NodeHandle n;
    
    const std::string topic = "/imu_correct";
    ROS_INFO("sub %s",topic.c_str());

    ros::Subscriber sub = n.subscribe(topic, 2000, imuHandler, ros::TransportHints().tcpNoDelay());
        // subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());

    ros::spin();

    return 0;
}
