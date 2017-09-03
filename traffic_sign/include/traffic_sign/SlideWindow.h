#ifndef __SLIDEWINDOW_H__
#define __SLIDEWINDOW_H__

class SlideWindow
{
public:
    std::vector<cv::Rect> getSlidingWindows(const cv::Size& img_size, const cv::Size& window_size, const cv::Size& step_size)
    {
        m_rect = cv::Rect(0, 0, window_size.width, window_size.height);
        m_window_size = window_size;
        m_step_size = step_size;
        m_img_size = img_size;
        
        std::vector<cv::Rect> windows;
        windows.push_back(m_rect);
        while (next())
        {
            windows.push_back(m_rect);
        }

        return windows;
    }

private:
    bool next()
    {
        if (m_rect.x + m_rect.width < m_img_size.width)
        {
            m_rect.x += m_step_size.width;
            return true;
        }
        else if (m_rect.y + m_rect.height < m_img_size.height)
        {
            m_rect.x = 0;
            m_rect.y += m_step_size.height;
            return true;                
        }
        
        return false;
    }
private:
    cv::Rect m_rect;
    cv::Size m_window_size;
    cv::Size m_step_size;
    cv::Size m_img_size;
};

#endif // __SLIDEWINDOW_H__
