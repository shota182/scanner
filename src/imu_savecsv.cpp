// csv_writer_node.cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <fstream>

std::ofstream csvFile;

void messageCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    csvFile << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << "," << msg->linear_acceleration.z << "," << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "csv_writer");
    ros::NodeHandle nh;

    // Open CSV file for writing
    csvFile.open("/home/sskr3/catkin_ws/src/scanner/data_csv/imu_2023-11-20-09-54-38.csv");

    // Subscribe to the topic
    ros::Subscriber sub = nh.subscribe("/imu/data_raw", 1000, messageCallback);

    ros::spin();

    // Close CSV file before exiting
    csvFile.close();

    return 0;
}
