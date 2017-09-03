#ifndef __LANEFINDERPUBLISHER_H__
#define __LANEFINDERPUBLISHER_H__

#include "EdgeDetector.h"
#include "LaneFinder.h"

bool FindLane(const cv::Mat& color_image, bool bshow_image = false)
{   
    cv::Mat image = color_image;

    EdgeDetector edge_task;
    if (!(edge_task << image))
        return false;

    LaneFinder lane_finder;

    cv::Point vanishing_point(image.cols / 2, image.rows / 2);
    lane_finder.setRange(cv::Range(30, 150));
    LaneFinder::LaneInfo lane_info;
    if (!lane_finder.find(edge_task.edge(), vanishing_point, lane_info))
        return false;

    if (bshow_image)
    {
        lane_finder.showHist();
    
        float left_rad = lane_info.left_line_rad;
        float right_rad = lane_info.right_line_rad;

        cv::Point left_v(1000 * cos(left_rad), 1000 * sin(left_rad));
        cv::Point right_v(1000 * cos(right_rad), 1000 * sin(right_rad));

        cv::line(image, vanishing_point, left_v + vanishing_point, CV_RGB(255, 0, 0), 8, 3);
        cv::line(image, vanishing_point, right_v + vanishing_point, CV_RGB(0, 255, 0), 8, 3);

        cv::imshow("img", image);
        cv::waitKey(1);

        const cv::Mat& polars = lane_finder.getPolarPoints();
        
        double r_max = 0;
        cv::minMaxLoc(polars.col(0), 0, &r_max);

        cv::Mat polar_image = cv::Mat::zeros(cv::Size(360, 500), CV_8UC1);
        for (int i = 0; i < polars.rows; i ++)
        {
            cv::Vec2f v;
            v[0] = polars.at<float>(i, 0); // polars.col(0).at<float>(i);
            v[1] = polars.at<float>(i, 1); // polars.col(1).at<float>(i);

            int degree = v[1] * 180 / CV_PI; //MIN(180 - 30, MAX((v[1] * 180.0 / CV_PI), 30));
            int radius = v[0] * 500.0 / r_max;

            //if (!(degree > 30 && degree < 150)) 
            //    continue;

            polar_image.at<char>(radius, degree) = 0xFF;
        }

        cv::imshow("polar", polar_image);
        cv::waitKey(1);
    }

    return true;
}




#endif