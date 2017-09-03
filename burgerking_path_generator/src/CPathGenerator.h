#ifndef __CPathGenerator_H_
#define __CPathGenerator_H_

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "tf/transform_listener.h"
#include "std_msgs/Header.h"
#include "burgerking_path_generator/pathpair.h"

#include <fstream>
#include <string>
#include <math.h>


#define pi 3.141592

enum occupancyValue{
    unknown = 0,
    empty,
    barrier
};


enum connection{
    up = 1,
    right,
    down,
    left,
    start,
    goal
};


struct Node
{
    int H; 
    connection from;

    std::vector<std::vector<std::pair<int, int> > > nodeComponent;      
};



class CPathGenerator
{
public: 
    CPathGenerator();
     ~CPathGenerator();

    // functions
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    void getRobotState();
 
    void searchGoal();

    void pathGenerate();

    int H_function(Node nd);

    bool NodeCapabilityCheck(Node nd);

    bool NewNodeCheck(Node nd);

    Node ChooseSelectNode(Node nd1, Node nd2);

    void SetPathNode(Node nd);

    void TransformPathNode(std::vector<std::pair<int, int> > pNode);

    void sendPathmsg();


    // variables 
    nav_msgs::MapMetaData info;
    std::vector<std::vector<int> > gridmap;

    tf::TransformListener listener;   
    tf::StampedTransform transform;

    float robotPos_x;
    float robotPos_y;
    float robot_angle;

    int robotGrid_x;
    int robotGrid_y;

    int Nd_rows;
    int Nd_cols;

    std::vector<std::pair<int, int> > goal_Candidate;

    Node StartNode;
    Node GoalNode;

    Node ParentNode;
    Node SelectNode;
    Node SubstituteGoalNode;

    std::vector<Node> ChildNode;
    std::vector<Node> OpenNode;
    std::vector<Node> ClosedNode;

    std::vector<std::pair<int, int> > PathNode;
    std::vector<std::pair<float, float> > T_PathNode;


    ros::NodeHandle handle;
    ros::Publisher path_pub;

};



#endif //__CPathGenerator_H_