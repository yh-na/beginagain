#ifndef __EDGEDETECTOR_H__
#define __EDGEDETECTOR_H__

#include <opencv2/opencv.hpp>

class EdgeDetector
{
public:
    bool operator << (const cv::Mat& mat)
    {
        try
        {
            cv::Mat gray;
            cv::cvtColor(mat, gray, CV_BGR2GRAY);
            cv::GaussianBlur(gray, gray, cv::Size(3, 3), 1.0, 1.0);
            cv::Mat edge;
            cv::Canny(gray, edge, 50, 200);
            m_edge = edge;
            m_gray = gray;
        }
        catch (cv::Exception& e)
        {
            std::cout << "[Class EdgeDetector]" << e.what();
            return false;
        }
    
        return true;
    }    

public:
    const cv::Mat& edge() const { return m_edge; }
    const cv::Mat& gray() const { return m_gray; }

private:
    cv::Mat m_edge;
    cv::Mat m_gray;
};



#endif