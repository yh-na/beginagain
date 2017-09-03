#include "Gradient.h"

bool GradientEdge::operator << (const cv::Mat& gray)
{
    try
    {
        cv::Size img_size = gray.size();

        cv::Mat blur_img;

        cv::GaussianBlur(gray, blur_img, cv::Size(5, 5), 1.0, 1.0);

        cv::Mat sobel[2];
        cv::Sobel(blur_img, sobel[0], CV_32F, 1, 0);
        cv::Sobel(blur_img, sobel[1], CV_32F, 0, 1);

        cv::Mat mag_mat(img_size, CV_32FC1);
        cv::Mat rad_mat(img_size, CV_32FC1);
        for (int r = 0; r < img_size.height; r ++)
        {
            for (int c = 0; c < img_size.width; c ++)
            {
                float dx = sobel[0].at<float>(r, c);
                float dy = sobel[1].at<float>(r, c);

                mag_mat.at<float>(r, c) = sqrt(dx*dx + dy*dy);
                rad_mat.at<float>(r, c) = atan2(dy, dx);
            }
        }

        cv::Mat edges = cv::Mat::zeros(img_size, CV_8UC1);

        int w_step = 1;
        int h_step = 1;

        int threshold_low = 50;
        int threshold_high = 200;

        for (int r = h_step; r < img_size.height - h_step; r ++)
        {
            for (int c = w_step; c < img_size.width - w_step; c ++)
            {
                float value = mag_mat.at<float>(r, c);
                float rad = rad_mat.at<float>(r,c); // 180.0f * rad_mat.at<float>(r,c) / CV_PI;
                int direction = (4 + ((int)round(rad / (CV_PI / 4.0)))) % 4;

                if (value > threshold_low)
                {
                    edges.at<uchar>(r,c) = MIN(value, 255);
                }

                // if (value > threshold_high)
                // {
                //     edges.at<uchar>(r,c) = 255;
                // }
                // else if (value > threshold_low)
                // {
                //     edges.at<uchar>(r,c) = 128;                    
                // }




                    

                // cv::Point dir_pt(round(cos(rad)), round(sin(rad)));
                
                // cv::Point dir_pts[4] = {
                //     cv::Point(1, 0),
                //     cv::Point(1, 1),
                //     cv::Point(0, 1),
                //     cv::Point(-1, 1)
                // };

                // float value_compare[2] {
                //     mag_mat.at<float>(cv::Point(r,c) + dir_pt), //dir_pts[direction]),
                //     mag_mat.at<float>(cv::Point(r,c) - dir_pt)//dir_pts[direction])
                // };

                // if (value > value_compare[0] + 0.1&&
                //     value > value_compare[1] + 0.1)
                // {
                //     if (value > threshold_high)
                //        edges.at<uchar>(r,c) = 255;
                //     else if (value > threshold_low)
                //         edges.at<uchar>(r,c) = 128;
                // }
            }
        }

        m_gradient_edges = edges;

        return true;
    }
    catch (cv::Exception& e)
    {
        std::cout << "[GradientEdge]" << e.what() << std::endl;
        return false;
    }
}
