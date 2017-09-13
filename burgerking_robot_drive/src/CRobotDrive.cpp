#include "CRobotDrive.h"


CRobotDrive::CRobotDrive():odom_x(0.0),odom_y(0.0),Bbox_x1(0.0),Bbox_x2(0.4),Bbox_y1(-0.1),Bbox_y2(0.1),
Rbox_x1(0.0),Rbox_x2(0.3),Rbox_y1(-0.2),Rbox_y2(0.2),FirstPathMsgFlag(false),
LinearVel(0.1),AngularVel(0.5)
{

}

CRobotDrive::~CRobotDrive()
{
    PathVector.clear();
    BarrierCheckbox.clear();
    RotationCheckbox.clear();
}




void CRobotDrive::processOdom(const nav_msgs::Odometry::ConstPtr &odomVal)
{

    odom_x = odomVal->pose.pose.position.x;
    odom_y = odomVal->pose.pose.position.y;

    printf("odom_x = %f, odom_y = %f\n",odom_x,odom_y);

    quat_x = odomVal->pose.pose.orientation.x;
    quat_y = odomVal->pose.pose.orientation.y;
    quat_z = odomVal->pose.pose.orientation.z;
    quat_w = odomVal->pose.pose.orientation.w;

    tf::Quaternion q(quat_x,quat_y, quat_z, quat_w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll, pitch, yaw);

    printf("odom_yaw = %f\n",yaw);


    if( PathVector.size() > 2 )
    {
        ScanDriveFlag = false;
        ROS_INFO("path drive mode\n");
        findToNode();

        // move robot current position -> ToNode 
        float theta = atan2f(ToNode.second-odom_y,ToNode.first-odom_x);    
        printf( "theta = %f\n" , theta);   

        
        if( abs( yaw - theta) < 0.17)
        {
            driveCmd(LinearVel , 0.0);
            printf("go\n");
        }
        else
        {
            if( yaw > 0 )
            {
                if( (theta > 0 && theta < yaw) || (theta > yaw-pi && theta < 0) ){
                    driveCmd(0.0 , -AngularVel);
                    printf(" go and turn right\n");
                }else{
                    driveCmd(0.0 , AngularVel);
                    printf(" go and turn left\n");
                }
            }
            else
            {
                if( (theta > yaw && theta < 0) || (theta > 0 && theta < yaw+pi)){
                    driveCmd(0.0 , AngularVel);
                    printf(" go and turn left\n");
                }else{
                    driveCmd(0.0 , -AngularVel);
                    printf(" go and turn right\n");
                }
            }
        }
    }
    else
    {
        if(FirstPathMsgFlag)
            ScanDriveFlag = true;
    }

}



void CRobotDrive::driveCmd(float control_linear_vel, float control_angular_vel)
{
    nDrivePub = handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    twist.linear.x = control_linear_vel;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;

    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = control_angular_vel;

    nDrivePub.publish(twist);

}



void CRobotDrive::findToNode()
{
    int nodeNum;
    float tmpLength;

    for(int i=0; i<PathVector.size(); i++)
    {      
        if( i==0 )
        {
            tmpLength = Odom2NodeLength(PathVector[0].first, PathVector[0].second);
            nodeNum = 0;
        }else
        {
            if( tmpLength > Odom2NodeLength(PathVector[i].first, PathVector[i].second)){
                tmpLength = Odom2NodeLength(PathVector[i].first, PathVector[i].second);
                nodeNum = i;
            }
        }
    }

    printf(" pathvector size = %d\n", (int)PathVector.size());
    printf("nodenum = %d\n",nodeNum);
    printf(" tmpLength = %f\n",tmpLength);


    if( tmpLength > 0.05 ){
        ToNode.first = PathVector[nodeNum].first;
        ToNode.second = PathVector[nodeNum].second;
    }else{
        ToNode.first = PathVector[nodeNum+1].first;
        ToNode.second = PathVector[nodeNum+1].second;
    }
     

    printf("To_x = %f, To_y = %f\n" ,ToNode.first, ToNode.second); 

}




float CRobotDrive::Odom2NodeLength(float x, float y)
{
    float length = sqrt((x-odom_x)*(x-odom_x) + (y-odom_y)*(y-odom_y));
    
    return length;
}



/*
void CRobotDrive::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    float robotPos_x = transform.getOrigin().x();
    float robotPos_y = transform.getOrigin().y();
    ROS_INFO("mapping data : x = %f, y = %f", robotPos_x, robotPos_y);

    float robot_angle = tf::getYaw(transform.getRotation());
    ROS_INFO("angle = %f", robot_angle);

}
*/



void CRobotDrive::processPath(const burgerking_path_generator::path::ConstPtr &msg)
{
    PathVector.clear();
    
    for(int i=0; i<msg->nodeVector.size(); i++){
        PathVector.push_back(std::make_pair(msg->nodeVector[i].nodeX,msg->nodeVector[i].nodeY ));
    }
    
    FirstPathMsgFlag = true;

    ROS_INFO("get path msg\n");

    for(int p=0; p<PathVector.size(); p++){
        printf("(%f,%f)\n",PathVector[p].first, PathVector[p].second);
    }        

}




void CRobotDrive::processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    if(FirstPathMsgFlag)
    {
        if(ScanDriveFlag)
        {
            ROS_INFO("scan drive mode\n");
            BarrierCheckbox.clear();
            RotationCheckbox.clear();

            projector.projectLaser(*scan_in, cloud);
            float scanPoint_x;
            float scanPoint_y;

            for(int i=0; i<cloud.points.size(); i++)
            {
                scanPoint_x = cloud.points[i].x;
                scanPoint_y = cloud.points[i].y;

                if( scanPoint_x >= Bbox_x1 && scanPoint_x <= Bbox_x2){
                    if( scanPoint_y > Bbox_y1 && scanPoint_y < Bbox_y2){
                        BarrierCheckbox.push_back(std::make_pair(scanPoint_x, scanPoint_y));
                    }
                }

                if( scanPoint_x >= Rbox_x1 && scanPoint_x <= Rbox_x2){
                    if( scanPoint_y >= Rbox_y1 && scanPoint_y <= Rbox_y2){
                        RotationCheckbox.push_back(std::make_pair(scanPoint_x, scanPoint_y));
                    }
                }
            }


            if( BarrierCheckbox.size() == 0 )
            {
                driveCmd(LinearVel , 0.0);
            }
            else
            {
                // find min point
                float min_x;

                for(int i = 0; i<BarrierCheckbox.size(); i++)
                {
                    if( i == 0 ){
                        min_x = BarrierCheckbox[i].first;
                    }else{
                        if( min_x > BarrierCheckbox[i].first ){
                            min_x = BarrierCheckbox[i].first;
                        }             
                    }
                }

                // move decision
                if( min_x < 0.2)
                {
                    int leftpart  = 0;
                    int rightpart = 0;

                    for(int p=0; p<RotationCheckbox.size(); p++)
                    {
                        if( RotationCheckbox[p].first < 0.2){
                            if( RotationCheckbox[p].second >0){
                                leftpart++;
                            }else{
                                rightpart++;
                            }
                        }
                    }

                    if( leftpart > rightpart){
                        driveCmd(0.0 , -AngularVel);  
                    }else{
                        driveCmd(0.0 , AngularVel);
                    }
                }
                else
                {
                    driveCmd(LinearVel , 0.0);
                }
            }
        }

    }

}

