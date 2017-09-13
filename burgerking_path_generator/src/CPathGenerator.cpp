#include "CPathGenerator.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "tf/transform_listener.h"
#include "std_msgs/Header.h"
#include "burgerking_path_generator/path.h"

#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <math.h>

CPathGenerator::CPathGenerator() : Nd_rows(5), Nd_cols(5), EntranceFlag(true),PathParam(1)
{
}

CPathGenerator::~CPathGenerator()
{
    gridmap.clear();
    goal_Candidate.clear();

    OpenNode.clear();
    ClosedNode.clear();
    ChildNode.clear();

    PathNode.clear();
}

void CPathGenerator::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // gridmap update //
    info = msg->info;
    rows = info.height;
    cols = info.width;

    gridmap.resize(rows);
    for (int i = 0; i < rows; i++)
    {
        gridmap[i].resize(cols);
    }

    int currCell = 0;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if (msg->data[currCell] == -1)
                gridmap[i][j] = unknown;
            else if (msg->data[currCell] == 100)
                gridmap[i][j] = barrier;
            else if (msg->data[currCell] == 0)
                gridmap[i][j] = empty;
            else
            {
            }
            currCell++;
        }
    }

    getRobotState();

    searchGoal();

    pathGenerate();

    sendPathmsg();


    for (int p = 0; p < PathNode.size(); p++)
    {
        gridmap[PathNode[p].first][PathNode[p].second] = 7;
    }

    // static int num = 0;
    // std::stringstream name;
    // name << "output" << num << ".txt";
    // std::ofstream outfile(name.str().c_str());

    // for(int i=0; i<rows; i++){
    //     for(int j=0; j<cols; j++){
    //         outfile << gridmap[i][j];
    //     }
    //     outfile << "\n";
    // }

    // outfile.close();
    // num++;


    cv::Size grid_size = cv::Size(cols, rows);
    cv::Mat grid_image = cv::Mat::zeros(grid_size, CV_8UC3);
    for (int h = 0; h < grid_size.height; h ++)
    {
        for (int w = 0; w < grid_size.width; w ++)
        {
            int value = gridmap[h][w];
            cv::Vec3b color(0, 0, 0);
            switch (value)
            {
            case 0: color = cv::Vec3b(64, 64, 64); break;
            case 1: color = cv::Vec3b(255, 255, 255); break;
            case 2: color = cv::Vec3b(0, 0, 0); break;
            case 7: color = cv::Vec3b(255, 0, 0); break;
            default:
                break;
            }

            grid_image.at<cv::Vec3b>(h, w) = color;
        }
    }

    cv::transpose(grid_image, grid_image);
    cv::flip(grid_image, grid_image, -1);
    cv::pyrUp(grid_image, grid_image);
    cv::imshow("grid_image", grid_image);
    cv::waitKey(1);


}



void CPathGenerator::getRobotState()
{
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    robotPos_x = transform.getOrigin().x();
    robotPos_y = transform.getOrigin().y();
    ROS_INFO("x = %f, y = %f", robotPos_x, robotPos_y);

    // robot orientation
    //robot_angle = tf::getYaw(transform.getRotation());
    //ROS_INFO("angle = %f", robot_angle);

    
    // [pixel]
    robotGrid_x = floor((robotPos_x*20.0)+0.5);
    robotGrid_y = floor((robotPos_y*20.0)+0.5);


    int tmp = robotGrid_x;
    robotGrid_x = robotGrid_y;
    robotGrid_y = tmp;


    int moveValue = floor(0.5+(0+5)/0.05);
    robotGrid_x += moveValue;
    robotGrid_y += moveValue;

    ROS_INFO("robotGrid_x = %d, robotGrid_y = %d", robotGrid_x, robotGrid_y);

    if(EntranceFlag){

        EntranceCell.first = robotGrid_x;
        EntranceCell.second = robotGrid_y;

        EntranceFlag = false;
    }


}



void CPathGenerator::searchGoal()
{
    int goal_row = EntranceCell.first;
    int goal_col = EntranceCell.second;
    float distance_ = 0.0;
    float distance = 0.0;

    for(int i=EntranceCell.first; i<(EntranceCell.first+41); i++){
        for(int j=EntranceCell.second; j<(EntranceCell.second+41); j++){
            
            if( gridmap[i][j] == empty )
            {
                float a = (i-EntranceCell.first)*0.05;
                float b = (j-EntranceCell.second)*0.05;

                distance = Length(a,b);

                if(distance > distance_)
                {
                    goal_row = i;
                    goal_col = j;

                    distance_ = distance;
                }
            }
        }
    }


    goal_Candidate.clear();

    // search goal -- confirm order
    // goal_Candidate.push_back(std::make_pair(goal_row, goal_col-4));
    // goal_Candidate.push_back(std::make_pair(goal_row, goal_col-3));
    // goal_Candidate.push_back(std::make_pair(goal_row, goal_col-2));
    // goal_Candidate.push_back(std::make_pair(goal_row, goal_col-1));
    // goal_Candidate.push_back(std::make_pair(goal_row, goal_col));


    goal_Candidate.push_back(std::make_pair(133, 132));
    goal_Candidate.push_back(std::make_pair(133, 133));
    goal_Candidate.push_back(std::make_pair(133, 134));
    goal_Candidate.push_back(std::make_pair(133, 135));
    goal_Candidate.push_back(std::make_pair(133, 136));


    GoalNode.H = 0;
    GoalNode.from = goal;

    GoalNode.nodeComponent.clear();
    GoalNode.nodeComponent.resize(Nd_rows);

    for (int i = 0; i < Nd_rows; i++)
    {
        GoalNode.nodeComponent[i].resize(Nd_cols);
    }

    //printf("Goal Node\n");

    for (int m = 0; m < Nd_rows; m++)
    {
        for (int n = 0; n < Nd_cols; n++)
        {

            GoalNode.nodeComponent[m][n].first = (goal_Candidate[m].first) - (Nd_rows - 3) + m;
            GoalNode.nodeComponent[m][n].second = goal_Candidate[n].second;

            //printf("(%d, %d)",GoalNode.nodeComponent[m][n].first,GoalNode.nodeComponent[m][n].second);
        }
        //printf("\n");
    }
}



void CPathGenerator::pathGenerate()
{
    OpenNode.clear();
    ClosedNode.clear();

    // 1. start node
    // robot position  & robot position is start node
    StartNode.H = 100000;
    StartNode.from = start;

    StartNode.nodeComponent.clear();
    StartNode.nodeComponent.resize(Nd_rows);

    for (int i = 0; i < Nd_rows; i++)
    {
        StartNode.nodeComponent[i].resize(Nd_cols);
    }

    //printf("Start Node\n");

    for (int m = 0; m < Nd_rows; m++)
    {
        for (int n = 0; n < Nd_cols; n++)
        {

            StartNode.nodeComponent[m][n].first = robotGrid_x - (Nd_rows / 2) + m;
            StartNode.nodeComponent[m][n].second = robotGrid_y - (Nd_cols / 2) + n;

            //printf("(%d, %d)",StartNode.nodeComponent[m][n].first,StartNode.nodeComponent[m][n].second);
        }
        //printf("\n");
    }

    // 2. put start node into closed node
    ClosedNode.push_back(StartNode);

    // 3. parent node = start node
    ParentNode.H = StartNode.H;
    ParentNode.from = StartNode.from;
    ParentNode.nodeComponent = StartNode.nodeComponent;

    while (1)
    {
        // 4. search child node & put into open node

        // (4) left
        Node LeftNode;

        LeftNode.from = right;

        LeftNode.nodeComponent.clear();
        LeftNode.nodeComponent.resize(Nd_rows);

        for (int i = 0; i < Nd_rows; i++)
        {
            LeftNode.nodeComponent[i].resize(Nd_cols);
        }

        //printf("Left node\n");

        for (int m = 0; m < Nd_rows; m++)
        {
            for (int n = 0; n < Nd_cols; n++)
            {

                LeftNode.nodeComponent[m][n].first = ParentNode.nodeComponent[m][n].first;
                LeftNode.nodeComponent[m][n].second = ParentNode.nodeComponent[m][n].second - PathParam;

                //printf("(%d, %d)",LeftNode.nodeComponent[m][n].first,LeftNode.nodeComponent[m][n].second);
            }
            //printf("\n");
        }

        LeftNode.H = H_function(LeftNode);

        if (LeftNode.H == 0)
        {
            printf("Path found!\n");
            PathNode.clear();
            ClosedNode.push_back(LeftNode);
            SetPathNode(LeftNode);
            break;
        }

        if (NodeCapabilityCheck(LeftNode))
        {
            if (NewNodeCheck(LeftNode))
            {
                OpenNode.push_back(LeftNode);
            }
        }


        // (1) up
        Node UpNode;

        UpNode.from = down;

        UpNode.nodeComponent.clear();
        UpNode.nodeComponent.resize(Nd_rows);

        for (int i = 0; i < Nd_rows; i++)
        {
            UpNode.nodeComponent[i].resize(Nd_cols);
        }

        //printf("Up node\n");

        for (int m = 0; m < Nd_rows; m++)
        {
            for (int n = 0; n < Nd_cols; n++)
            {

                UpNode.nodeComponent[m][n].first = ParentNode.nodeComponent[m][n].first - PathParam;
                UpNode.nodeComponent[m][n].second = ParentNode.nodeComponent[m][n].second;

                //printf("(%d, %d)",UpNode.nodeComponent[m][n].first,UpNode.nodeComponent[m][n].second);
            }
            //printf("\n");
        }

        UpNode.H = H_function(UpNode);

        if (UpNode.H == 0)
        {
            printf("Path found!\n");
            PathNode.clear();
            ClosedNode.push_back(UpNode);
            SetPathNode(UpNode);
            break;
        }

        if (NodeCapabilityCheck(UpNode))
        {
            if (NewNodeCheck(UpNode))
            {
                OpenNode.push_back(UpNode);
            }
        }

        // (2) right
        Node RightNode;

        RightNode.from = left;

        RightNode.nodeComponent.clear();
        RightNode.nodeComponent.resize(Nd_rows);

        for (int i = 0; i < Nd_rows; i++)
        {
            RightNode.nodeComponent[i].resize(Nd_cols);
        }

        //printf("Right node\n");

        for (int m = 0; m < Nd_rows; m++)
        {
            for (int n = 0; n < Nd_cols; n++)
            {

                RightNode.nodeComponent[m][n].first = ParentNode.nodeComponent[m][n].first;
                RightNode.nodeComponent[m][n].second = ParentNode.nodeComponent[m][n].second + PathParam;

                //printf("(%d, %d)",RightNode.nodeComponent[m][n].first,RightNode.nodeComponent[m][n].second);
            }
            //printf("\n");
        }

        RightNode.H = H_function(RightNode);

        if (RightNode.H == 0)
        {
            printf("Path found!\n");
            PathNode.clear();
            ClosedNode.push_back(RightNode);
            SetPathNode(RightNode);
            break;
        }

        if (NodeCapabilityCheck(RightNode))
        {
            if (NewNodeCheck(RightNode))
            {
                OpenNode.push_back(RightNode);
            }
        }

        // (3) down
        Node DownNode;

        DownNode.from = up;

        DownNode.nodeComponent.clear();
        DownNode.nodeComponent.resize(Nd_rows);

        for (int i = 0; i < Nd_rows; i++)
        {
            DownNode.nodeComponent[i].resize(Nd_cols);
        }

        //printf("Down node\n");

        for (int m = 0; m < Nd_rows; m++)
        {
            for (int n = 0; n < Nd_cols; n++)
            {

                DownNode.nodeComponent[m][n].first = ParentNode.nodeComponent[m][n].first + PathParam;
                DownNode.nodeComponent[m][n].second = ParentNode.nodeComponent[m][n].second;

                //printf("(%d, %d)",DownNode.nodeComponent[m][n].first,DownNode.nodeComponent[m][n].second);
            }
            //printf("\n");
        }

        DownNode.H = H_function(DownNode);

        if (DownNode.H == 0)
        {
            printf("Path found!\n");
            PathNode.clear();
            ClosedNode.push_back(DownNode);
            SetPathNode(DownNode);
            break;
        }

        if (NodeCapabilityCheck(DownNode))
        {
            if (NewNodeCheck(DownNode))
            {
                OpenNode.push_back(DownNode);
            }
        }





        if (OpenNode.size() == 0)
        {
            printf("Substitutue path!\n");
            PathNode.clear();

            for (int p = 0; p < ClosedNode.size(); p++)
            {
                if (p == 0)
                {
                    SubstituteGoalNode = ClosedNode[0];
                }
                else
                {
                    SubstituteGoalNode = ChooseSelectNode(SubstituteGoalNode, ClosedNode[p]);
                }
            }

            SetPathNode(SubstituteGoalNode);
            break;
        }

        // 5. loop open node & decide parent node (H is the smallest)
        for (int p = 0; p < OpenNode.size(); p++)
        {
            if (p == 0)
            {
                SelectNode = OpenNode[0];
            }
            else
            {
                SelectNode = ChooseSelectNode(SelectNode, OpenNode[p]);
            }
        }

        // 6. put into the closed node
        ClosedNode.push_back(SelectNode);

        std::vector<Node>::iterator it;
        for (it = OpenNode.begin(); it != OpenNode.end();)
        {
            if (it->nodeComponent[Nd_rows / 2][Nd_cols / 2] == SelectNode.nodeComponent[Nd_rows / 2][Nd_cols / 2])
            {
                it = OpenNode.erase(it);
            }
            else
            {
                it++;
            }
        }

        // 7. parent node = select node
        ParentNode = SelectNode;

        printf("-------------------------------\n");
    }

    for (int a = 0; a < PathNode.size(); a++)
    {
        printf("(%d, %d)\n", PathNode[a].first, PathNode[a].second);
    }

    TransformPathNode(PathNode);

    for (int a = 0; a < T_PathNode.size(); a++)
    {
        printf("(%f, %f)\n", T_PathNode[a].first, T_PathNode[a].second);
    }
}



int CPathGenerator::H_function(Node nd)
{
    int GoalCenter_x = GoalNode.nodeComponent[Nd_rows / 2][Nd_cols / 2].first;
    int GoalCenter_y = GoalNode.nodeComponent[Nd_rows / 2][Nd_cols / 2].second;

    int ndCenter_x = nd.nodeComponent[Nd_rows / 2][Nd_cols / 2].first;
    int ndCenter_y = nd.nodeComponent[Nd_rows / 2][Nd_cols / 2].second;

    int x_count = abs(GoalCenter_x - ndCenter_x);
    int y_count = abs(GoalCenter_y - ndCenter_y);

    return x_count + y_count;
}



bool CPathGenerator::NodeCapabilityCheck(Node nd)
{
    for (int i = 0; i < Nd_rows; i++)
    {
        for (int j = 0; j < Nd_cols; j++)
        {
            if (gridmap[nd.nodeComponent[i][j].first][nd.nodeComponent[i][j].second] == barrier ||
                gridmap[nd.nodeComponent[i][j].first][nd.nodeComponent[i][j].second] == unknown)
            {
                return false;
            }
        }
    }

    return true;
}

bool CPathGenerator::NewNodeCheck(Node nd)
{
    int ndCenter_x = nd.nodeComponent[Nd_rows / 2][Nd_cols / 2].first;
    int ndCenter_y = nd.nodeComponent[Nd_rows / 2][Nd_cols / 2].second;

    for (int i = 0; i < ClosedNode.size(); i++)
    {
        if (ndCenter_x == ClosedNode[i].nodeComponent[Nd_rows / 2][Nd_cols / 2].first &&
            ndCenter_y == ClosedNode[i].nodeComponent[Nd_rows / 2][Nd_cols / 2].second)
        {

            return false;
        }
    }

    return true;
}



Node CPathGenerator::ChooseSelectNode(Node nd1, Node nd2)
{
    if (nd1.H <= nd2.H)
    {
        return nd1;
    }
    else
    {
        return nd2;
    }
}



void CPathGenerator::SetPathNode(Node nd)
{
    PathNode.push_back(std::make_pair(nd.nodeComponent[Nd_rows/2][Nd_cols/2].first,nd.nodeComponent[Nd_rows/2][Nd_cols/2].second));

    if(nd.from == up)
    {
        for(int i=0; i<ClosedNode.size(); i++){
            if( ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].first == nd.nodeComponent[Nd_rows/2][Nd_cols/2].first - PathParam &&
                ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].second == nd.nodeComponent[Nd_rows/2][Nd_cols/2].second)
                {
                    SetPathNode(ClosedNode[i]);
                    break;
                }
        }
    }
    else if(nd.from == right)
    {
        for(int i=0; i<ClosedNode.size(); i++){
            if( ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].first == nd.nodeComponent[Nd_rows/2][Nd_cols/2].first  &&
                ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].second == nd.nodeComponent[Nd_rows/2][Nd_cols/2].second + PathParam)
                {
                    SetPathNode(ClosedNode[i]);
                    break;
                }
        }
    } 
    else if(nd.from == down)
    {
        for(int i=0; i<ClosedNode.size(); i++){
            if( ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].first == nd.nodeComponent[Nd_rows/2][Nd_cols/2].first + PathParam &&
                ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].second == nd.nodeComponent[Nd_rows/2][Nd_cols/2].second)
                {
                    SetPathNode(ClosedNode[i]);
                    break;
                }
        }        
    }
    else if(nd.from == left)
    {
        for(int i=0; i<ClosedNode.size(); i++){
            if( ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].first == nd.nodeComponent[Nd_rows/2][Nd_cols/2].first  &&
                ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].second == nd.nodeComponent[Nd_rows/2][Nd_cols/2].second - PathParam)
                {
                    SetPathNode(ClosedNode[i]);
                    break;
                }
        }
    }else{}


}



void CPathGenerator::TransformPathNode(std::vector<std::pair<int, int>> pNode)
{
    T_PathNode.clear();
    std::vector<std::pair<int, int>>::reverse_iterator riter;

    float T_x;
    float T_y;

    for (riter = pNode.rbegin(); riter != pNode.rend();)
    {
        T_x = (riter->second)*0.05;
        T_y = (riter->first)*0.05;

        T_x = T_x - 5.0;
        T_y = T_y - 5.0;

        T_PathNode.push_back(std::make_pair(T_x, T_y));

        riter++;
    }
}



void CPathGenerator::sendPathmsg()
{
    path_pub = handle.advertise<burgerking_path_generator::path>("path_msg", 1);
    burgerking_path_generator::node nodeMsg;
    burgerking_path_generator::path pathMsg;

    for(int i=0; i<T_PathNode.size(); i++){

        nodeMsg.nodeX = T_PathNode[i].first;
        nodeMsg.nodeY = T_PathNode[i].second;

        pathMsg.nodeVector.push_back(nodeMsg);
    }

    path_pub.publish(pathMsg);
    ROS_INFO("send path ---- size:%d\n",(unsigned int)T_PathNode.size());
}
