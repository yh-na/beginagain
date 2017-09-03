#include "LaneFinder.h"

LaneFinder::LaneFinder()
    : m_angle_range(0, 360)
{

}

LaneFinder::LaneFinder(cv::Range angle_range)
    : m_angle_range(angle_range)
{

}

LaneFinder::~LaneFinder()
{

}

void LaneFinder::setRange(cv::Range angle_range)
{
    m_angle_range = angle_range;
}

bool LaneFinder::find(const cv::Mat& edge_image, const cv::Point vanishing_point, LaneInfo& detected_lane)
{
    try
    {
        prepare(edge_image, vanishing_point);
        const cv::Mat& polar_vector = getPolarPoints();

        cv::Mat rad_vector = polar_vector.col(1);

        float rad_range[2] = {0.f, CV_PI * 2.f};
        const float *rad_hist_range = {rad_range};

        int rad_hist_size = 360;

        cv::Mat rad_hist, rho_hist;
        cv::calcHist(&rad_vector, 1, NULL, cv::Mat(), rad_hist, 1, &rad_hist_size, &rad_hist_range, true, true);

        m_rad_hist = rad_hist;

        int mid = 90; // (m_angle_range.start + m_angle_range.end) / 2;
        int left = m_angle_range.start;
        int right = m_angle_range.end;

        cv::Range left_line_range(left, mid);
        cv::Range right_line_range(mid, right);

        cv::Mat left_line = cv::Mat(rad_hist.rowRange(left_line_range));
        cv::Mat right_line = cv::Mat(rad_hist.rowRange(right_line_range));

        int max_left_line_angle_idx, max_right_line_angle_idx;
        cv::minMaxIdx(left_line, 0, 0, 0, &max_left_line_angle_idx);
        cv::minMaxIdx(right_line, 0, 0, 0, &max_right_line_angle_idx);

        max_left_line_angle_idx += left_line_range.start;
        max_right_line_angle_idx += right_line_range.start;

        LaneInfo lane_info;
        lane_info.left_line_rad = (float)max_left_line_angle_idx * (CV_PI / 180.0f);
        lane_info.right_line_rad = (float)max_right_line_angle_idx * (CV_PI / 180.0f);
        lane_info.vanishing_point = vanishing_point;

        detected_lane = lane_info;

        return true;    
    }
    catch (cv::Exception& e)
    {
        std::cout << "[Class LaneFinder]" << e.what();
        return false;
    }
}

void LaneFinder::prepare(const cv::Mat& edge_image, const cv::Point vanishing_point)
{
    cv::Size img_size = edge_image.size();
    std::vector<cv::Vec2f> edge_point_vector;
    for (int h = 0; h < img_size.height; h ++)
    {
        for (int w = 0; w < img_size.width; w ++)
        {
            cv::Point pt(w,h);
            if (edge_image.at<char>(pt) != 0)
            {
                edge_point_vector.push_back(cv::Vec2f(w, h));
            }
        }
    }

    cv::Mat edge_mat = cv::Mat(edge_point_vector).reshape(1);
    m_edge_points = edge_mat;
    
    std::vector<cv::Mat> polar_vectors(2);
    cv::Mat x_array = cv::Mat(edge_mat.col(0)) - vanishing_point.x;
    cv::Mat y_array = cv::Mat(edge_mat.col(1)) - vanishing_point.y;
    cv::cartToPolar(x_array, y_array, polar_vectors[0], polar_vectors[1]);

    cv::Mat polar_points;
    cv::merge(polar_vectors, polar_points);

    m_polar_points = polar_points.reshape(1);
}

void LaneFinder::showHist()
{
    cv::Mat rad_hist_image = cv::Mat::zeros(cv::Size(360, 500), CV_8UC3);

    cv::Mat rad_hist;
    
    cv::normalize(m_rad_hist, rad_hist, 0, 1, cv::NORM_MINMAX);

    for( int i = 0; i < rad_hist.rows; i++ )
    {
        cv::Point p0(i-1, 500 * rad_hist.at<float>(i-1));
        cv::Point p1(i, 500 * rad_hist.at<float>(i));

        cv::line(rad_hist_image, p0, p1, CV_RGB(0, 255, 0));
    }

    cv::line(rad_hist_image, cv::Point(90, 0), cv::Point(90, 1000), CV_RGB(255, 0, 0), 8, 3);

    cv::imshow("rad_hist_imag", rad_hist_image);
    cv::waitKey(1);
}