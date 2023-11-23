#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

std::string camera_device;  // カメラデバイスのパラメータ

bool laser = true; // trueなら/image_laser、falseなら/image_odomに画像をパブリッシュ

int main(int argc, char** argv) {
    ros::init(argc, argv, "capture");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");  // プライベートネームスペースのノードハンドル

    // パラメータからカメラデバイスの取得
    nh_private.param<std::string>("camera_device", camera_device, "/dev/video0");

    ros::Publisher image_laser_pub, image_odom_pub;

    image_laser_pub = nh.advertise<sensor_msgs::Image>("/image_laser", 1);
    image_odom_pub = nh.advertise<sensor_msgs::Image>("/image_odom", 1);

    cv::VideoCapture cap(camera_device);  // カメラデバイスのオープン
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open the camera.");
        return -1;
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(30);  // ループの実行レートを設定

    while (ros::ok()) {
        cap >> frame;  // フレームの取得

        if (frame.empty()) {
            ROS_ERROR("Failed to capture an image.");
            break;
        }

        // OpenCVのMatをROSのImageメッセージに変換
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        // 画像をパブリッシュ
        if (laser) {
            image_laser_pub.publish(msg);
        } else {
            image_odom_pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
