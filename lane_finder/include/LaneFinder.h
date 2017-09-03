#ifndef __LANEFINDER_H__
#define __LANEFINDER_H__

#include <opencv2/opencv.hpp>

class LaneFinder
{
public:
    struct LaneInfo
    {
        cv::Point vanishing_point;
        float left_line_rad;
        float right_line_rad;
    };

public:
    LaneFinder();
    LaneFinder(cv::Range angle_range);    
    ~LaneFinder();

public:
    void setRange(cv::Range angle_range);

public:
    bool find(const cv::Mat& edge_image, const cv::Point vanishing_point, LaneInfo& detected_lane);
    
    void showHist();
public:
    const cv::Mat& getEdgePoints() const { return m_edge_points; }
    const cv::Mat& getPolarPoints() const { return m_polar_points; }

protected:
    void prepare(const cv::Mat& edge_image, const cv::Point vanishing_point);

private:
    cv::Mat m_polar_points;
    cv::Mat m_edge_points;

private:
    cv::Mat m_rad_hist;
    cv::Range m_angle_range;
};

#endif // __LANEFINDER_H__