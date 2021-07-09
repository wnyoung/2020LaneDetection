
#define _USE_MATH_DEFINES

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <stdio.h>
#include <cmath>

using namespace cv;
using namespace std;



class LaneDetector {
private:
    double img_size;
    double img_center;
    bool left_flag = false;
    bool right_flag = false;
    cv::Point right_b;
    double right_m;
    cv::Point left_b;
    double left_m;

public:
    cv::Mat cvtHSL(cv::Mat inputImage);
    cv::Mat deNoise(cv::Mat inputImage);
    cv::Mat edgeDetector(cv::Mat img_noise);
    cv::Mat mask(cv::Mat img_edges);
    std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);
    std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges);
    std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage);
    int plotLane(cv::Mat inputImage, std::vector<cv::Point> lane);
};


int main(int argc, char** argv)
{
    Mat image, gray_image;
    Mat edgeImage, orginalImageWithHoughLines;

    //VideoCapture cap("clip1.mp4");
    VideoCapture cap("clip4.mp4");

    if (!cap.isOpened())
        return -1;

    LaneDetector lanedetector; 
    cv::Mat videoFrame;
    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;
    std::string turn;


    while (true) {
        if (!cap.read(videoFrame))
            break;
        resize(videoFrame, videoFrame, Size(videoFrame.size().width / 2, videoFrame.size().height / 2), 0, 0, INTER_LINEAR);




        img_denoise = lanedetector.deNoise(videoFrame);
        //imshow("img_denoise", img_denoise);

        //videoFrame = lanedetector.cvtHSL(videoFrame);


        img_edges = lanedetector.edgeDetector(img_denoise);
        //imshow("img_edges", img_edges);

        img_mask = lanedetector.mask(img_edges);
        //imshow("img_mask", img_mask);
        
        lines = lanedetector.houghLines(img_mask);
        

        if (!lines.empty()) {

            left_right_lines = lanedetector.lineSeparation(lines, img_edges);

            lane = lanedetector.regression(left_right_lines, videoFrame);

            lanedetector.plotLane(videoFrame, lane);

            
        }

        
        if (waitKey(1) == 27)
            break;

    }
    return 0;
}