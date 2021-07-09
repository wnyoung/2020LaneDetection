
#define _USE_MATH_DEFINES

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <string>
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

cv::Mat LaneDetector::cvtHSL(cv::Mat inputImage) {
    //HSL
    Mat hsl;
    cvtColor(inputImage, hsl, COLOR_RGB2HLS);
    imshow("COLOR_RGB2HLS", hsl);

    Mat  mask, masked, image;

    Mat white_mask, white_image;
    Scalar lower_white = Scalar(round(0 / 2), round(0.75 * 255), round(0.00 * 255));
    Scalar upper_white = Scalar(round(360 / 2), round(1.00 * 255), round(0.30 * 255));
    inRange(hsl, lower_white, upper_white, white_mask);

    Mat blue_mask, blue_image;
    Scalar lower_blue = Scalar(round(40 / 2), round(0.00 * 255), round(0.35 * 255));
    Scalar upper_blue = Scalar(round(60 / 2), round(1.00 * 255), round(1.00 * 255));
    inRange(hsl, lower_blue, upper_blue, blue_mask);

    mask = white_mask + blue_mask;
    bitwise_or(inputImage, inputImage, image, mask);
    bitwise_and(inputImage, inputImage, white_image, blue_mask);

    //imshow("img", videoFrame);
    imshow("image", image);
    imshow("mask", mask);

    return image;
}




cv::Mat LaneDetector::deNoise(cv::Mat inputImage) {
    cv::Mat output;

    cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);

    return output;
}


cv::Mat LaneDetector::edgeDetector(cv::Mat img_noise) {
    /*
    Mat contours;
    Canny(img_noise, contours, 50, 400);
    return contours;
    */
        
    cv::Mat output;
    cv::Mat kernel;
    cv::Point anchor;
    


    cv::cvtColor(img_noise, output, cv::COLOR_RGB2GRAY);

    Canny(output, output, 50, 100);
    //imshow("output", output);
    return output;
    
    
}


cv::Mat LaneDetector::mask(cv::Mat img_edges) {
    int wsize = img_edges.size().width;
    int hsize = img_edges.size().height;
    cv::Mat output;
    cv::Mat mask = cv::Mat::zeros(img_edges.size(), img_edges.type());
    cv::Point pts[4] = {
        //Point(w,h)
       cv::Point(wsize * 0 / 10,    hsize * 8 / 10),
       cv::Point(wsize * 4 / 10,     hsize * 6 / 10),
       cv::Point(wsize * 6 / 10 ,    hsize * 6 / 10),
       cv::Point(wsize * 10 / 10,     hsize * 8 / 10)
    };


    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255,0,0));
    cv::bitwise_and(img_edges, mask, output);

    return output;
}


std::vector<cv::Vec4i> LaneDetector::houghLines(cv::Mat img_mask) {
    std::vector<cv::Vec4i> line;

    HoughLinesP(img_mask, line, 1, CV_PI / 180, 50, 20, 400);
    //cv::HoughLines(img_mask, line, 1, CV_PI / 180, 200);

    return line;
}

std::vector<std::vector<cv::Vec4i> > LaneDetector::lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
    std::vector<std::vector<cv::Vec4i> > output(2);
    size_t j = 0;
    cv::Point ini;
    cv::Point fini;

    std::vector<double> slopes;
    std::vector<cv::Vec4i> selected_lines;
    std::vector<cv::Vec4i> right_lines, left_lines;
    double slope_thresh_min = 0.5;
    double slope_thresh_max = 3;


    for (auto i : lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);
        

        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y)) / (static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);


        if (std::abs(slope) > slope_thresh_min && abs(slope)< slope_thresh_max) {
            slopes.push_back(slope);
            selected_lines.push_back(i);
        }
        
    }
    /*
    for (auto i : selected_lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);
        cv::line(img_edges, ini, fini, cv::Scalar(255), 5, 8);
        imshow("line!!", img_edges);
    }
    */


    img_center = static_cast<double>((img_edges.cols / 2));
    while (j < selected_lines.size()) {
        ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
        fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);


        if (slopes[j] > 0 && fini.x > img_center && ini.x > img_center) {
            right_lines.push_back(selected_lines[j]);
            right_flag = true;
        }
        else if (slopes[j] < 0 && fini.x < img_center && ini.x < img_center) {
            left_lines.push_back(selected_lines[j]);
            left_flag = true;
        }
        j++;
    }

    output[0] = right_lines;
    output[1] = left_lines;

    //for (auto i : right_lines) {
    //    ini = cv::Point(i[0], i[1]);
    //    fini = cv::Point(i[2], i[3]);
    //    cv::line(img_edges, ini, fini, cv::Scalar(255), 5, 8);
    //    imshow("line!!", img_edges);
    //}

    return output;
}


std::vector<cv::Point> LaneDetector::regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage) {
    std::vector<cv::Point> output(4);
    cv::Point ini;
    cv::Point fini;
    cv::Point ini2;
    cv::Point fini2;
    cv::Vec4d right_line;
    cv::Vec4d left_line;
    std::vector<cv::Point> right_pts;
    std::vector<cv::Point> left_pts;
    double slope_thresh_min = 0.5;
    double slope_thresh_max = 1.5;


    if (right_flag == true) {
        for (auto i : left_right_lines[0]) {
            ini = cv::Point(i[0], i[1]);
            fini = cv::Point(i[2], i[3]);

            right_pts.push_back(ini);
            right_pts.push_back(fini);
        }

        if (right_pts.size() > 0) {

            cv::fitLine(right_pts, right_line, 2, 0, 0.01, 0.01);
            right_m = right_line[1] / right_line[0];


            cout << "RM: " << right_m << endl;
            right_b = cv::Point(right_line[2], right_line[3]);

            //cout << "RM: "<<right_m << endl;
            //right_b = cv::Point(right_line[2], right_line[3]);

            //cv::line(inputImage, right_b, Point(right_line[2] + right_line[0], right_line[3]+ right_line[1]), cv::Scalar(255,0,0), 5, 8);
            //imshow("line!!", inputImage);
        }

    }

    if (left_flag == true) {
        for (auto j : left_right_lines[1]) {
            ini2 = cv::Point(j[0], j[1]);
            fini2 = cv::Point(j[2], j[3]);

            left_pts.push_back(ini2);
            left_pts.push_back(fini2);
        }

        if (left_pts.size() > 0) {
            cv::fitLine(left_pts, left_line, 2, 0, 0.01, 0.01);

            left_m = left_line[1] / left_line[0];

            //if (abs(left_m) > 0.5 && abs(left_m) < 3) {
                left_m = left_line[1] / left_line[0];
                cout << "LM: " << left_m << endl;
                left_b = cv::Point(left_line[2], left_line[3]);
            //}
            //else {
            //    left_m = -0.7;
            //    cout << "LM: " << left_m << endl;
            //    left_b = cv::Point(inputImage.size().width / 3, inputImage.size().height);
            //}


            //cout << "LM: " << left_m << endl;
            //left_b = cv::Point(left_line[2], left_line[3]);
            //line(inputImage, Point(0,0), left_b, cv::Scalar(255, 0, 0), 5, 8);
        }

    }
    //if (left_right_lines[0].size() == 0) {
    //    right_m = 0.8;
    //    cout << "RM: " << right_m << endl;
    //    right_b = cv::Point(inputImage.size().width * 2 / 3, inputImage.size().height);
    //}
    //if (left_right_lines[1].size() == 0) {
    //    left_m = -0.8;
    //    cout << "LM: " << left_m << endl;
    //    left_b = cv::Point(inputImage.size().width / 3, inputImage.size().height);
    //}



    int ini_y = inputImage.rows;
    double fin_y = (left_m * right_m / (right_m - left_m))*((right_b.x - left_b.x) 
        + (left_b.y / left_m) - (right_b.y / right_m));
    double fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;


    double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
    double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;


    output[0] = cv::Point(right_ini_x, ini_y);
    output[1] = cv::Point(fin_x, fin_y);
    output[2] = cv::Point(left_ini_x, ini_y);
    output[3] = cv::Point(fin_x, fin_y);


    //차선 넘어갔는지 감지
    if (abs(right_m) > slope_thresh_max && right_ini_x < inputImage.size().width * 2 / 3) {
        String text1 = "right lane invasion!!";
        Size textSize1 = getTextSize(      //Size는 텍스트의 width, heigh 이 멤버 변수
            text1,
            1,      //폰트 종류
            4,      //확대 축소 비율
            1, 0);
        Point textOrg1((inputImage.cols - textSize1.width) / 10, 60);
        putText(inputImage, text1, textOrg1, 1, 4, Scalar(0, 255, 255), 2, 8);
    }
    else if (abs(left_m) > slope_thresh_max && left_ini_x > inputImage.size().width * 1 / 3) {
        String text2 = "Left lane invasion!!";
        Size textSize2 = getTextSize(      //Size는 텍스트의 width, heigh 이 멤버 변수
            text2,
            1,      //폰트 종류
            4,      //확대 축소 비율
            1, 0);
        Point textOrg2((inputImage.cols - textSize2.width) / 10, 60);
        putText(inputImage, text2, textOrg2, 1, 4, Scalar(0, 255, 255), 2, 8);
    }


    return output;
}



int LaneDetector::plotLane(cv::Mat inputImage, std::vector<cv::Point> lane) {
    std::vector<cv::Point> poly_points;
    cv::Mat output;

    double slope_r = (static_cast<double>(lane[0].y) - static_cast<double>(lane[1].y)) / (static_cast<double>(lane[0].x) - (static_cast<double>(lane[1].x)));
    //cout << slope_r << endl;
    //if (abs(slope_r) < 0.5) {

    //    right_m = 0.7;
    //    cout << "RM: " << right_m << endl;
    //    right_b = cv::Point(inputImage.size().width * 2 / 3, inputImage.size().height);
    //}


    double slope_l = (static_cast<double>(lane[2].y) - static_cast<double>(lane[3].y)) / (static_cast<double>(lane[2].x) - (static_cast<double>(lane[3].x)));



    inputImage.copyTo(output);
    poly_points.push_back(lane[2]);
    poly_points.push_back(lane[0]);
    poly_points.push_back(lane[1]);
    poly_points.push_back(lane[3]);
    cv::fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), 8, 0);
    cv::addWeighted(output, 0.3, inputImage, 1.0 - 0.3, 0, inputImage);

    cv::line(inputImage, lane[0], lane[1], cv::Scalar(0, 255, 255), 5, 8);
    cv::line(inputImage, lane[2], lane[3], cv::Scalar(0, 255, 255), 5, 8);


    cv::namedWindow("Lane");
    cv::imshow("Lane", inputImage);
    return 0;
}