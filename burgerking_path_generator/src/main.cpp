#include "ros/ros.h"
#include "CPathGenerator.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_generator");

    CPathGenerator pGenerator;

    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("map", 100, &CPathGenerator::mapCallback, &pGenerator);


    ros::spin();
    return 0;
}