#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Dense>

// publish laser position by integrate acceleration twice
class LaserPointCloudCalculator {
public:
    LaserPointCloudCalculator() : nh_("~"){
        // Initialize variables
        previous_timestamp_ = ros::Time::now();
        laser_subscriber_ = nh_.subscribe("/image_laser/binarization", 10, &LaserPointCloudCalculator::imageCallback, this);
        caminfo_subscriber_ = nh_.subscribe("/camera_info", 1, &LaserPointCloudCalculator::cameraInfoCallback, this);
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
        camera_model_.fromCameraInfo(camera_info_msg); // set camera param
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& binary_image_msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(binary_image_msg, sensor_msgs::image_encodings::MONO8);

            sensor_msgs::PointCloud points2d_msg;
            points2d_msg.header = binary_image_msg->header;
            sensor_msgs::PointCloud points3d_msg;
            points3d_msg.header = binary_image_msg->header; // フレーム違うので修正が必要
            std::vector<cv::Point> points;
            cv::findNonZero(cv_ptr->image, points); // get 255 points

            // Eigen::MatrixXd型のポジションベクトルを取得
            Eigen::MatrixXd pos_vector(3, points.size());
            for (size_t i = 0; i < points.size(); ++i) {
                cv::Point3d point3d = camera_model.projectPixelTo3dRay(cv::Point2d(points[i].x, points[i].y));
                pos_vector.col(i) << point3d.x, point3d.y, point3d.z; // vector(z=1)
            }

            std::vector<geometry_msgs::Point32> points3d = get_points3d(pos_vector, a_, b_, c_, d_); // calc intersection
            points3d_msg.points = points3d;

            points2d_pub_.publish(points2d_msg);
            points3d_pub_.publish(points3d_msg);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    std::vector<geometry_msgs::Point32> get_points3d(const Eigen::MatrixXd& pos_vector, double a, double b, double c, double d) {
        std::vector<geometry_msgs::Point32> points;
        Eigen::Vector3d vector_normal(a_, b_, c_); // normal vector of laser_plane

        for (int i = 0; i < pos_vector.cols(); ++i) {
            double t = -d / vector_normal.dot(pos_vector.col(i));

            geometry_msgs::Point32 point;
            point.x = pos_vector(0, i) * t;
            point.y = pos_vector(1, i) * t;
            point.z = pos_vector(2, i) * t;

            points.push_back(point);
        }

        return points;
    }

    ros::NodeHandle nh_;
    ros::Subscriber laser_subscriber_;
    ros::Subscriber caminfo_subscriber_;
    ros::Publisher points2d_pub_;
    ros::Publisher points3d_pub_;

    image_geometry::PinholeCameraModel camera_model_;
    YAML::Node config = YAML::LoadFile("laser.yaml"); // get param from laser.yaml
    YAML::Node laserEquation = config["laser_equation"];
    double a_ = laserEquation["a"].as<double>();
    double b_ = laserEquation["b"].as<double>();
    double c_ = laserEquation["c"].as<double>();
    double d_ = laserEquation["d"].as<double>();
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_cloud");
    LaserPointCloudCalculator laser_calculator;

    ros::spin();

    return 0;
}
