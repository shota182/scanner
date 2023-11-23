#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

class ExtKalmanFilter {
public:
    ExtKalmanFilter() : {
        // initialization
        previous_timestamp_ = ros::Time::now();
        imu_->header.stamp = 0;
        mag_->header.stamp = 0;
        myu_observation_ = Eigen::Vector3d::Zero();
        Sigma_observation_ = Eigen::Matrix3d::Identity();
        R_ = Eigen::Matrix3d::Identity() * 0.01;
        Q_ = Eigen::Matrix3d::Identity() * 0.1;

        ros::NodeHandle nh("~");
        imu_sub_ = nh.subscribe("/imu/data_raw", 10, &ExtKalmanFilter::imuCallback, this);
        mag_sub_ = nh.subscribe("/imu/mag", 10, &ExtKalmanFilter::magCallback, this);
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data_ekf", 1);
    }

    // Callback function for IMU data
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        if(imu_->header.stamp < mag_->header.stamp) imu_ = imu_msg;
        ros::Time current_timestamp = imu_->header.stamp;
        dt_ = (current_timestamp - previous_timestamp_).toSec();
        previous_timestamp_ = current_timestamp;
        if(imu_->header.stamp == mag_->header.stamp) calc_ekf();
    }

    // Callback function for magnetometer data
    void magCallback(const sensor_msgs::MagneticField::ConstPtr& mag_msg) {
        if(mag_->header.stamp < imu_->header.stamp) mag_ = mag_msg;
        if(mag_->header.stamp == imu_->header.stamp) calc_ekf();
    }

private:
    Eigen::Quaterniond eulerToQuaternion(const Eigen::Vector3d& euler) {
        Eigen::Quaterniond q;
        return Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
    }
    geometry_msgs::Quaternion eigenQuaternionToMsg(const Eigen::Quaterniond& eigenQuaternion) {
        geometry_msgs::Quaternion q;
        q.w = eigenQuaternion.w();
        q.x = eigenQuaternion.x();
        q.y = eigenQuaternion.y();
        q.z = eigenQuaternion.z();
        return q;
    }

    sensor_msgs::Imu setimu(const sensor_msgs::Imu::ConstPtr& imu){
        sensor_msgs::Imu setimu;
        setimu.header                         = imu->header;
        setimu.orientation                    = imu->orientation;
        setimu.orientation_covariance         = imu->orientation_covariance;
        setimu.angular_velocity               = imu->angular_velocity;
        setimu.angular_velocity_covariance    = imu->angular_velocity_covariance;
        setimu.linear_acceleration            = imu->linear_acceleration;
        setimu.linear_acceleration_covariance = imu->linear_acceleration_covariance;
        return setimu;
    }
    
    void calc_ekf(){
        sensor_msgs::Imu ekfImu;
        ekfImu = setimu(imu_);
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Vector3d myu_prediction;
        Eigen::Matrix3d Sigma_prediction;
        Eigen::Matrix3d g; // prediction
        Eigen::Vector3d h; // observation
        Eigen::Matrix3d G; // prediction Jacob
        Eigen::Matrix3d H; // observation Jacob
        Eigen::Matrix3d K; // Kalman Gain
        Eigen::Vector3d acceleration;
        Eigen::Vector3d omega;
        Eigen::Vector3d magnetic;
        acceleration.x() = imu_->linear_acceleration.x;
        acceleration.y() = imu_->linear_acceleration.y;
        acceleration.z() = imu_->linear_acceleration.z;
        omega.x() = imu_->angular_velocity.x;
        omega.y() = imu_->angular_velocity.y;
        omega.z() = imu_->angular_velocity.z;
        magnetic.x() = mag_->magnetic_field.x;
        magnetic.y() = mag_->magnetic_field.y;
        magnetic.z() = mag_->magnetic_field.z;

        // prediction
        g << 1, tan(myu_observation_.y()) * sin(myu_observation_.x()), tan(myu_observation_.y()) * cos(myu_observation_.x()),
             0,                             cos(myu_observation_.x()),                            -sin(myu_observation_.x()),
             0, sin(myu_observation_.x()) / cos(myu_observation_.y()), cos(myu_observation_.x()) / cos(myu_observation_.y());
        myu_prediction = myu_observation_ + g * (omega * dt_);

        G << 1+(omega.x()+omega.y()*tan(myu_observation_.y())*cos(myu_observation_.x())-omega.z()*tan(myu_observation_.y())*sin(myu_observation_.x()))*dt_,                                                     (omega.y()*sin(myu_observation_.x())/(cos(myu_observation_.y())*cos(myu_observation_.y()))+omega.z()*cos(myu_observation_.x())/(cos(myu_observation_.y())*cos(myu_observation_.y())))*dt_, 0,
                                                                            (-omega.y()*sin(myu_observation_.x())-omega.z()*cos(myu_observation_.x()))*dt_,                                                                                                                                                                                                                                             1, 0,
                         (omega.y()*cos(myu_observation_.x())/cos(myu_observation_.y())-omega.z()*sin(myu_observation_.x())/cos(myu_observation_.y()))*dt_, (omega.y()*sin(myu_observation_.x())*sin(myu_observation_.y())/(cos(myu_observation_.y())*cos(myu_observation_.y()))+omega.x()*cos(myu_observation_.x())*sin(myu_observation_.y())/(cos(myu_observation_.y())*cos(myu_observation_.y())))*dt_, 1;
        Sigma_prediction = G * Sigma_observation_ * G.transpose() + R_;

        // observation
        h << atan(-acceleration.y()/(-acceleration.z())), 
             atan(acceleration.x()/sqrt((acceleration.y()*acceleration.y())+(acceleration.z()*acceleration.z()))),
             0;
        h(2) = atan((magnetic.x()*cos(h(1))+magnetic.y()*sin(h(1))*sin(h(0))+magnetic.z()*sin(h(1))*cos(h(0)))/(magnetic.y()*cos(h(0))-magnetic.z()*sin(h(0)))); // yaw
        H = Eigen::Matrix3d::Identity();
        K = Sigma_prediction * H.transpose() * (H.transpose() * Sigma_prediction * H + Q_).inverse();

        myu_observation_ = myu_prediction + K *(h - H * myu_prediction);
        Sigma_observation_ = (I - K * H) * Sigma_prediction;

        // pub
        geometry_msgs::Quaternion q = eigenQuaternionToMsg(eulerToQuaternion(myu_observation_));
        ekfImu.orientation = q;
        imu_pub_.publish(ekfImu);
    }

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber mag_sub_;
    ros::Publisher imu_pub_;

    sensor_msgs::Imu::ConstPtr imu_;
    sensor_msgs::MagneticField::ConstPtr mag_;

    double dt_;
    ros::Time previous_timestamp_;

    Eigen::Vector3d myu_observation_; // myu
    Eigen::Matrix3d Sigma_observation_; // sigma

    Eigen::Matrix3d R_; // prediction error
    Eigen::Matrix3d Q_; // observation error

    // YAML::Node config = YAML::LoadFile("/home/sskr3/catkin_ws/src/scanner/config/imu.yaml"); // param from imu.yaml
    // YAML::Node Acceleration = config["acceleration"]; // acceleration param
    // double accel_e0x_ = Acceleration["e0"]["x"].as<double>();
    // double accel_e0y_ = Acceleration["e0"]["y"].as<double>();
    // double accel_e0z_ = Acceleration["e0"]["z"].as<double>();
    // double accel_e1x_ = Acceleration["e1"]["x"].as<double>();
    // double accel_e1y_ = Acceleration["e1"]["y"].as<double>();
    // double accel_e1z_ = Acceleration["e1"]["z"].as<double>();
    // YAML::Node Angular_velocity = config["angular_velocity"]; // angular_velocity param
    // double angle_e0x_ = Angular_velocity["e0"]["x"].as<double>();
    // double angle_e0y_ = Angular_velocity["e0"]["y"].as<double>();
    // double angle_e0z_ = Angular_velocity["e0"]["z"].as<double>();
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf");
    ExtKalmanFilter ekf;
    ros::spin();
    return 0;
}
