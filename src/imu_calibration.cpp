#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <fstream>
#include <iostream>
#include <ctime>

// データ保存用のファイルストリーム
std::ofstream csvFile;

// データ保存フラグ
int count = 0;
bool saveData = false;
bool loop = true;

std::map<char, std::pair<bool, bool>> key_mapping = {
    {'q', {false, false}}, {'d', {false, true}},
    {'s', {true, true}}
};

// IMUデータのコールバック関数
// void imuCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
//     if (saveData) {
//         // データをCSVファイルに保存
//         std::cout << "Saved data to CSV..." << std::endl;
//         csvFile << msg->magnetic_field.x << ",";
//         csvFile << msg->magnetic_field.y << ",";
//         csvFile << msg->magnetic_field.z << std::endl;
//     }
// }
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (saveData) {
        // データをCSVファイルに保存
        std::cout << "Saved data to CSV..." << std::endl;
        csvFile << msg->linear_acceleration.x << ",";
        csvFile << msg->linear_acceleration.y << ",";
        csvFile << msg->linear_acceleration.z << ",";
        csvFile << msg->angular_velocity.x << ",";
        csvFile << msg->angular_velocity.y << ",";
        csvFile << msg->angular_velocity.z << std::endl;
        // count++;
        // if(count >= 100) saveData = false;
    }
}

void keysCallback(const std_msgs::String::ConstPtr& msg) {
    if (key_mapping.find(msg->data[0]) != key_mapping.end()) {
        std::cout << msg->data << std::endl;
        saveData = key_mapping[msg->data[0]].first;
        loop = key_mapping[msg->data[0]].second;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_calibration");
    ros::NodeHandle nh;
    // 現在の時刻を取得
    time_t now = time(0);

    // tm 構造体に変換
    tm *localTime = localtime(&now);

    // 年月日時分秒を取得
    int year = 1900 + localTime->tm_year;
    int month = 1 + localTime->tm_mon;
    int day = localTime->tm_mday;
    int hour = localTime->tm_hour;
    int minute = localTime->tm_min;

    std::stringstream filename;
    filename << "/home/sskr3/catkin_ws/src/scanner/data_csv/for_calibraiton/imu_data_" << year << "-" << month << "-" << day << "_" << hour << "-" << minute << ".csv";

    // CSVファイルを開く
    csvFile.open(filename.str());

    // IMUデータのサブスクライバーを作成
    ros::Subscriber imuSub = nh.subscribe("/imu/data_raw", 10, imuCallback);
    ros::Subscriber keys_sub = nh.subscribe("/key", 1, keysCallback);
    // ros::Subscriber imuSub = nh.subscribe("/imu/mag", 1000, imuCallback);

    std::cout << "Press 's' to start saving data to CSV, 'q' to quit." << std::endl;

	// ros::Rate loop_rate(100);
    while (ros::ok() && loop) {
        // std::cout << "Loop now" << std::endl;
        ros::spinOnce();
    }

    // ファイルを閉じる
    csvFile.close();

    return 0;
}
