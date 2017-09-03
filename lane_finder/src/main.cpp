#include "ros/ros.h"
#include "LaneFinderPublisher.h"
#include "std_msgs/Header.h"
#include "lane_finder/lane_finder.h"
#include "sensor_msgs/Image.h"

int main(int argc, char** argv)
{
    // TO DO :
    // 1) Subscribe camera image from sensor_msgs
    // 2) Publish lane_info

    // ros::NodeHandle nh;
    // ros::Publisher  publisher;
    // ros::Subscriber subscriber;

    cv::Mat test_image = cv::imread("");
    FindLane(test_image, true);

    cv::waitKey();
 
    return 0;
}

