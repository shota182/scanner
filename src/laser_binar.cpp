#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/video/background_segm.hpp>  

sensor_msgs::CameraInfo camera_info;  // グローバル変数として宣言
sensor_msgs::ImagePtr binary_msg;
cv::Mat previousImage;

cv::Mat frame;
// MOG2（Mixture of Gaussians）バックグラウンドモデルの設定
cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2 = cv::createBackgroundSubtractorMOG2();

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg) {
    camera_info = *info_msg;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // ROSイメージメッセージをOpenCVイメージに変換
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat currentImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        // BGR画像を分割
        std::vector<cv::Mat> channels;
        cv::split(currentImage, channels);
        // R成分のみを抽出
        currentImage = channels[2];  // 0: Blue, 1: Green, 2: Red
        
        if (previousImage.empty()) {
            previousImage = currentImage.clone();
            return;
        }

        // 背景差分の計算
        // cv::Mat diffImage;
        // cv::absdiff(previousImage, currentImage, diffImage);

        // 背景差分の計算
        cv::Mat fgMask;
        pMOG2->apply(currentImage, fgMask, 0.05);

        // 現在の画像を前の画像にコピー
        currentImage.copyTo(previousImage);

        // // グレースケール変換
        // cv::Mat grayImage;
        // cv::cvtColor(cv_ptr->image, grayImage, cv::COLOR_BGR2GRAY);

        // // 二値化
        // cv::Mat binaryImage;
        // cv::threshold(grayImage, binaryImage, 128, 255, cv::THRESH_BINARY);

        // 二値化した画像をROSイメージメッセージに変換してパブリッシュ
        // sensor_msgs::ImagePtr binary_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", binaryImage).toImageMsg();
        binary_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", fgMask).toImageMsg();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char* argv[])
{
    // This must be called before anything else ROS-related
    ros::init(argc, argv, "laser_binar");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // イメージトランスポートハンドラの作成
    image_transport::ImageTransport it(nh);
    image_transport::Publisher binarization_pub = it.advertise("/image_binarization", 1);

    // カメラの画像トピックのサブスクライバを作成
    ros::Subscriber sub = nh.subscribe("/image", 1, imageCallback);
    // ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);

    // OpenCVウィンドウの初期化
    // cv::namedWindow("Binary Image", cv::WINDOW_AUTOSIZE);

    // ROSループ
	ros::Rate loop_rate(100);
    while (ros::ok())
	{
		ros::spinOnce();
		
        // 二値化した画像をROSイメージメッセージに変換してパブリッシュ
        // sensor_msgs::ImagePtr binary_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", binaryImage).toImageMsg();
        binarization_pub.publish(binary_msg);
		loop_rate.sleep();
	}

    // OpenCVウィンドウの削除
    // cv::destroyWindow("Binary Image");

    return 0;
}
