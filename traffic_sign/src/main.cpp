#include <ros/ros.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <traffic_sign/SlideWindow.h>


using namespace cv;

Mat _inRange(Mat input, int min, int max)
{
    Mat returnImg = Mat::zeros(input.rows, input.cols, CV_8UC1);

    for(int h = 0; h < input.cols; h ++)
        for(int w = 0; w < input.rows; w ++)
        {
            if(input.data[(h * input.rows) + w] < max && input.data[(h * input.rows) + w] > min)
                returnImg.data[(h * input.rows) + w] = 255;
        }

    return returnImg;

}

Point _sampleMatching(Mat& rgbImg, Mat originImg, Mat templateImg)
{
    int result_cols =  originImg.cols - templateImg.cols + 1;
    int result_rows = originImg.rows - templateImg.rows + 1;

    Mat result;
    result.create( result_rows, result_cols, CV_32FC1 );

    int match_method = CV_TM_SQDIFF_NORMED;
    matchTemplate( originImg, templateImg, result,  match_method);
    //CV_TM_SQDIFF_NORMED 3
    //CV_TM_CCORR_NORMED 3
    //CV_TM_CCOEFF_NORMED 3
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
      { matchLoc = minLoc; }
    else
      { matchLoc = maxLoc; }
    //imshow( "re", result );

    /// Show me what you got
    rectangle( rgbImg, matchLoc, Point( matchLoc.x + templateImg.cols , matchLoc.y + templateImg.rows ), Scalar::all(0), 2, 8, 0 );
    return matchLoc;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "traffic_sign");
    ros::NodeHandle nh;

    ///////read image
    char* imageName = "/home/user/catkin_ws/src/traffic_sign/image.png";
    Mat readImg;
    readImg = imread( imageName, 1 );
    //Mat readGrayImg;
    Mat readHSVImg;
    vector<Mat> reasHSVVec;

    Mat readYellowImg;
    Mat readBlueImg;

    //cvtColor( readImg, readGrayImg, CV_BGR2GRAY );
    cvtColor(readImg, readHSVImg, CV_BGR2HSV );
    split(readHSVImg, reasHSVVec);

    readBlueImg = _inRange(reasHSVVec[0], 100, 120); // 100, 110
    readYellowImg = _inRange(reasHSVVec[0], 20, 35);
    blur( readBlueImg, readBlueImg, Size(3,3) );
    blur( readYellowImg, readYellowImg, Size(3,3) );



    ///////read sample1
    char* imageName1 = "/home/user/catkin_ws/src/traffic_sign/sample1_bin.jpg";
    Mat readSampleImg1;
    readSampleImg1 = imread( imageName1, 1 );
    cvtColor( readSampleImg1, readSampleImg1, CV_BGR2GRAY );


    cv::putText( readImg, "Parking", _sampleMatching(readImg, readBlueImg, readSampleImg1), 2, 1, Scalar::all(0) );

    ///////read sample2
    char* imageName2 = "/home/user/catkin_ws/src/traffic_sign/sample2_bin.jpg";
    Mat readSampleImg2;
    readSampleImg2 = imread( imageName2, 1 );
    cvtColor( readSampleImg2, readSampleImg2, CV_BGR2GRAY );

    cv::putText( readImg, "Train", _sampleMatching(readImg, readYellowImg, readSampleImg2), 2, 1, Scalar::all(0) );

    ///////read sample3
    char* imageName3 = "/home/user/catkin_ws/src/traffic_sign/sample3_bin.jpg";
    Mat readSampleImg3;
    readSampleImg3 = imread( imageName3, 1 );
    cvtColor( readSampleImg3, readSampleImg3, CV_BGR2GRAY );

    cv::putText( readImg, "Tunnel", _sampleMatching(readImg, readYellowImg, readSampleImg3), 2, 1, Scalar::all(0) );

    ///////Template matching
    /// Create the result matrix

    imshow( "result_window", readImg );
    //imshow( "result_ye", readYellowImg );
    //imshow( "result_bl", readBlueImg );


    waitKey(0);

    ros::shutdown();
    return 0;
}
