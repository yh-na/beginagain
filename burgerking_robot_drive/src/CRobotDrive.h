#ifndef __CRobotDrive_H__
#define __CRobotDrive_H__

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"
#include "burgerking_path_generator/path.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "nav_msgs/OccupancyGrid.h"


#include <math.h>


#define min(a,b) ((a) < (b) ? (a) : (b))
#define pi 3.141592


class CRobotDrive
{
public:
    CRobotDrive();
     ~CRobotDrive();

    // functions
    void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan_in);
    
    void processPath(const burgerking_path_generator::path::ConstPtr &cmd);

    void driveCmd(float control_linear_vel, float control_angular_vel);

    void processOdom(const nav_msgs::Odometry::ConstPtr &odomVal);

    //void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    void findToNode();

    float Odom2NodeLength(float x, float y);


    // variables
    geometry_msgs::Twist twist;

    ros::NodeHandle handle;
    ros::Publisher nDrivePub;

    float LinearVel;
    float AngularVel;

    std::vector<std::pair<float, float> > PathVector;

    tf::TransformListener listener;   
    tf::StampedTransform transform;

    float odom_x;
    float odom_y;

    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;

    std::pair<float, float> ToNode;

    bool ScanDriveFlag;
    bool FirstPathMsgFlag;

    laser_geometry::LaserProjection projector;
   
    sensor_msgs::PointCloud cloud;

    std::vector<std::pair<float, float> > BarrierCheckbox;
    std::vector<std::pair<float, float> > RotationCheckbox;

    const float Bbox_x1;
    const float Bbox_x2;
    const float Bbox_y1;
    const float Bbox_y2;

    const float Rbox_x1;
    const float Rbox_x2;
    const float Rbox_y1;
    const float Rbox_y2;




    


};




#endif //__CRobotDrive_H__