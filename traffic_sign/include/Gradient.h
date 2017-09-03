#ifndef __GRADIENT_H__
#define __GRADIENT_H__

#include <opencv2/opencv.hpp>

class GradientEdge
{
public:
    bool operator << (const cv::Mat& gray);

public:
    cv::Mat_<cv::Vec2f> m_gradient;
    cv::Mat m_gradient_edges;
};

#endif // __GRADIENT_H__
