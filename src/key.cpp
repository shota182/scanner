#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termios.h>

ros::Publisher keys_pub;

void publishKey(char key) {
    std_msgs::String key_msg;
    key_msg.data = std::string(1, key);
    keys_pub.publish(key_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "key_publisher");
    ros::NodeHandle nh;

    keys_pub = nh.advertise<std_msgs::String>("/key", 1);

    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    char key;
    while (ros::ok()) {
        if (read(STDIN_FILENO, &key, 1) != 1) {
            continue;
        }

        ROS_INFO("Key pressed: %c", key);
        publishKey(key);

        if (key == 'q') {
            break;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return 0;
}
