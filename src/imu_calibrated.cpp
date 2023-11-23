#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

class ImuCalibrationNode {
public:
    ImuCalibrationNode() {
        // ROS関連の初期化
        ros::NodeHandle nh("~");
        imuRawSub = nh.subscribe("/imu/data_raw", 1, &ImuCalibrationNode::imuRawCallback, this);
        imuCalibratedPub = nh.advertise<sensor_msgs::Imu>("/imu/data_calibrated", 1);
    }

    void imuRawCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // データをキャリブレーション
        sensor_msgs::Imu calibratedImu = calibrateImu(msg);

        // キャリブレーション後のデータをpublish
        imuCalibratedPub.publish(calibratedImu);
    }

private:
    sensor_msgs::Imu calibrateImu(const sensor_msgs::Imu::ConstPtr& rawImu) {
        sensor_msgs::Imu calibratedImu ;

        calibratedImu.header = rawImu->header;
        calibratedImu.angular_velocity_covariance = rawImu->angular_velocity_covariance;
        calibratedImu.linear_acceleration_covariance = rawImu->linear_acceleration_covariance;

        calibratedImu.linear_acceleration.x = (rawImu->linear_acceleration.x - accel_e0x_) / (1 + accel_e1x_);
        calibratedImu.linear_acceleration.y = (rawImu->linear_acceleration.y - accel_e0y_) / (1 + accel_e1y_);
        calibratedImu.linear_acceleration.z = (rawImu->linear_acceleration.z - accel_e0z_) / (1 + accel_e1z_);
        calibratedImu.angular_velocity.x = rawImu->angular_velocity.x - angle_e0x_;
        calibratedImu.angular_velocity.y = rawImu->angular_velocity.y - angle_e0y_;
        calibratedImu.angular_velocity.z = rawImu->angular_velocity.z - angle_e0z_;
        return calibratedImu;
    }

    ros::NodeHandle nh_;
    ros::Subscriber imuRawSub;
    ros::Publisher imuCalibratedPub;

    YAML::Node config = YAML::LoadFile("/home/sskr3/catkin_ws/src/scanner/config/imu.yaml"); // param from imu.yaml
    YAML::Node Acceleration = config["acceleration"]; // acceleration param
    double accel_e0x_ = Acceleration["e0"]["x"].as<double>();
    double accel_e0y_ = Acceleration["e0"]["y"].as<double>();
    double accel_e0z_ = Acceleration["e0"]["z"].as<double>();
    double accel_e1x_ = Acceleration["e1"]["x"].as<double>();
    double accel_e1y_ = Acceleration["e1"]["y"].as<double>();
    double accel_e1z_ = Acceleration["e1"]["z"].as<double>();
    YAML::Node Angular_velocity = config["angular_velocity"]; // angular_velocity param
    double angle_e0x_ = Angular_velocity["e0"]["x"].as<double>();
    double angle_e0y_ = Angular_velocity["e0"]["y"].as<double>();
    double angle_e0z_ = Angular_velocity["e0"]["z"].as<double>();

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_calibrated");
    ImuCalibrationNode imuCalibrationNode;
    ros::spin();
    return 0;
}
