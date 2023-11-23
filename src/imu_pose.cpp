#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Twist.h"
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

// publish imu position by integrate acceleration twice
class ImuDisplacementCalculator {
public:
    ImuDisplacementCalculator() : nh_("~"), imu_subscriber_(nh_.subscribe("/imu/data", 10, &ImuDisplacementCalculator::imuCallback, this)) {
        // Initialize variables
        previous_timestamp_ = ros::Time::now();
        displacement_.x() = 0.0;
        displacement_.y() = 0.0;
        displacement_.z() = 0.0;
        velocity_.x() = 0.0;
        velocity_.y() = 0.0;
        velocity_.z() = 0.0;
        // displacement_.setZero();
        // velocity_.setZero();

        // Set the frame IDs
        frame_id_ = "world";
        child_frame_id_ = "odom"; // cut orientation
        imuPosePub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_odom", 1);
        imuVeloPub_ = nh_.advertise<geometry_msgs::Twist>("/imu/velocity", 1);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        // if(delay <= 0){
        // Calculate time difference
        ros::Time current_timestamp = imu_msg->header.stamp;
        double dt = (current_timestamp - previous_timestamp_).toSec();
        previous_timestamp_ = current_timestamp;

        // publish imu_odom
        sensor_msgs::Imu poseImu;
        poseImu.header = imu_msg->header;
        poseImu.header.frame_id = "odom";
        poseImu.angular_velocity_covariance = imu_msg->angular_velocity_covariance;
        poseImu.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;
        Eigen::Vector3d accel = adjust_axis(imu_msg->linear_acceleration);
        poseImu.linear_acceleration.x = accel.x();
        poseImu.linear_acceleration.y = accel.y();
        poseImu.linear_acceleration.z = accel.z();
        poseImu.angular_velocity = imu_msg->angular_velocity;


        geometry_msgs::Twist velocity_msg;
        velocity_msg.linear.x = velocity_.x();
        velocity_msg.linear.y = velocity_.y();
        velocity_msg.linear.z = velocity_.z();
        velocity_msg.angular.x = -accel.x();
        velocity_msg.angular.y = -accel.y();
        velocity_msg.angular.z = -accel.z();
        
        imuPosePub_.publish(poseImu);
        imuVeloPub_.publish(velocity_msg);

        // Calculate displacement using double integration
        displacement_ += calc_position(imu_msg->linear_acceleration, dt);
        orientation_ = ros_to_eigen_quat(imu_msg->orientation);

        // Publish the TF transform
        publishTFTransform(current_timestamp);
        // } else {
        //     delay-=1;
        //     displacement_.x() = 0.0;
        //     displacement_.y() = 0.0;
        //     displacement_.z() = 0.0;
        //     velocity_.x() = 0.0;
        //     velocity_.y() = 0.0;
        //     velocity_.z() = 0.0;
        //     // displacement_.setZero();
        //     // velocity_.setZero();
        // }
    }

private:

    // Function to convert from ROS geometry_msgs::Quaternion
    Eigen::Quaterniond ros_to_eigen_quat(const geometry_msgs::Quaternion & q) {
        return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    }

    // Function to convert from ROS Eigen::Quaterniond
    geometry_msgs::Quaternion eigen_to_ros_quat(const Eigen::Quaterniond & q) {
        geometry_msgs::Quaternion quat;
        quat.x = q.x();
        quat.y = q.y();
        quat.z = q.z();
        quat.w = q.w();
        return quat;
    }

    Eigen::Matrix3d quat_to_rotation(const Eigen::Quaterniond& q) {
        Eigen::Matrix3d rotation;
        rotation(0,0) = 2*q.w()*q.w()+2*q.x()*q.x()-1;
        rotation(0,1) = 2*q.x()*q.y()+2*q.z()*q.w();
        rotation(0,2) = 2*q.x()*q.z()-2*q.y()*q.w();
        rotation(1,0) = 2*q.x()*q.y()-2*q.z()*q.w();
        rotation(1,1) = 2*q.w()*q.w()+2*q.y()*q.y()-1;
        rotation(1,2) = 2*q.y()*q.z()+2*q.x()*q.w();
        rotation(2,0) = 2*q.x()*q.z()+2*q.y()*q.w();
        rotation(2,1) = 2*q.y()*q.z()-2*q.x()*q.w();
        rotation(2,2) = 2*q.w()*q.w()+2*q.z()*q.z()-1;
        return rotation;
    }


    Eigen::Vector3d adjust_axis(const geometry_msgs::Vector3& linear_acceleration) {
        // calculate rotation matrix from quaternion
        Eigen::Vector3d acceleration_imu_link(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z); // odom
        Eigen::Matrix3d rotation_matrix = orientation_.toRotationMatrix(); // odom -> imu_link
        Eigen::Vector3d acceleration = orientation_ * acceleration_imu_link; // convert from acceleration(odom) to acceleration(world)
        // acceleration.z() += g_; // cut gravity
        return acceleration;
    }

    Eigen::Vector3d calc_position(const geometry_msgs::Vector3& linear_acceleration, double dt) {
        // calculate rotation matrix from quaternion
        Eigen::Vector3d acceleration_imu_link(linear_acceleration.x, linear_acceleration.y, linear_acceleration.z); // odom
        Eigen::Matrix3d rotation_matrix = orientation_.toRotationMatrix(); // odom -> imu_link
        Eigen::Vector3d acceleration = orientation_ * acceleration_imu_link; // convert from acceleration(odom) to acceleration(world)
        acceleration = -acceleration;
        acceleration.z() += g_; // cut gravity
        // Double integration: v = u + at, s = ut + 0.5at^2
        velocity_ += acceleration * dt; // Update velocity(world)
        Eigen::Vector3d displacement_change = 0.5 * acceleration * dt * dt + velocity_ * dt; // Update displacement(world)

        return displacement_change;
    }

    // void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
    // {
    //     tf::Quaternion quat;
    //     quaternionMsgToTF(geometry_quat, quat);
    //     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // }

    void publishTFTransform(const ros::Time& timestamp) {

        // Create TF transform
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = timestamp;
        transformStamped.header.frame_id = frame_id_;
        transformStamped.child_frame_id = child_frame_id_;
        transformStamped.transform.translation.x = displacement_.x();
        transformStamped.transform.translation.y = displacement_.y();
        transformStamped.transform.translation.z = displacement_.z();
        transformStamped.transform.rotation = eigen_to_ros_quat(orientation_);

        // Convert Quaternion directly
        // tf2::Quaternion quaternion;
        // quaternion.setRPY(0, 0, 0);  // Set roll, pitch, and yaw to zero for identity quaternion
        // tf2::convert(quaternion, transformStamped.transform.rotation);

        // transformStamped.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());
        // tf2::toMsg(tf2::Quaternion::getIdentity(), transformStamped.transform.rotation);

        // Publish TF transform
        tf_broadcaster_.sendTransform(transformStamped);
    }

    ros::NodeHandle nh_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher imuPosePub_;
    ros::Publisher imuVeloPub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Time previous_timestamp_;
    Eigen::Vector3d displacement_;
    Eigen::Quaterniond orientation_;
    // geometry_msgs::Quaternion orientation_;
    Eigen::Vector3d velocity_;

    std::string frame_id_;
    std::string child_frame_id_;
    double g_ = 9.80665;
    // int delay = 0; // 最初のデータを捨てる
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_pose");
    ImuDisplacementCalculator imu_calculator;

    ros::spin();

    return 0;
}
