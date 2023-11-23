#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class LaserBinarizaiton {
public:
    LaserBinarizaiton() : nh_("~"){
        // previous_timestamp_ = ros::Time::now();
        laser_subscriber_ = nh_.subscribe("/image_laser", 10, &LaserBinarizaiton::imageCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat prebinary_image; // processing before binarization
            prebinary_image = cv_ptr->image;
            // cv::cvtColor(cv_ptr->image, prebinary_image, cv::COLOR_BGR2GRAY);

            cv::Mat binary_image; // binarization by color recognition
            if(colortype_ == "bgr") cv::inRange(prebinary_image, cv::Scalar(minValues_[0], minValues_[1], minValues_[2]), cv::Scalar(maxValues_[0], maxValues_[1], maxValues_[2]), binary_image);
            else if(colortype_ == "rgb") cv::inRange(prebinary_image, cv::Scalar(minValues_[2], minValues_[1], minValues_[0]), cv::Scalar(maxValues_[2], maxValues_[1], maxValues_[0]), binary_image);

            cv::Mat denoised_image; // processing after binarization
            cv::medianBlur(binary_image, denoised_image, 3); // median filter

            // 二値化画像をROSイメージメッセージに変換
            sensor_msgs::ImagePtr binarization_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", denoised_image).toImageMsg();

            // 二値化画像を新しいトピックにパブリッシュ
            ros::NodeHandle nh;
            image_transport::ImageTransport it(nh);
            image_transport::Publisher binarization_pub = it.advertise("/image_laser/binarization", 1);
            binarization_pub.publish(binarization_msg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber laser_subscriber_;

    YAML::Node config = YAML::LoadFile("laser.yaml"); // param from laser.yaml
    YAML::Node laserColor = config["laser_color"]; // laser_color param
    std::string colortype_ = laserColor["colortype"].as<std::string>();
    int rows_ = laserColor["rows"].as<int>();
    int cols_ = laserColor["cols"].as<int>();
    std::vector<double> minValues_ = laserColor["min"].as<std::vector<double>>();
    std::vector<double> maxValues_ = laserColor["max"].as<std::vector<double>>();

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_binarization");
    LaserBinarizaiton laser_binar;

    ros::spin();

    return 0;
}
