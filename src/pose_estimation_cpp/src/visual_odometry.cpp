#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

#define CAPTURE_WIDTH  640
#define CAPTURE_HEIGHT 480
#define DISPLAY_WIDTH  CAPTURE_WIDTH
#define DISPLAY_HEIGHT CAPTURE_HEIGHT
#define FRAME_RATE     30
#define FLIP_METHOD    0

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) 
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}


cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(80, true);

int main(int argc, char * argv[])
{   
    std::string pipeline = gstreamer_pipeline(CAPTURE_WIDTH,CAPTURE_HEIGHT,DISPLAY_WIDTH,DISPLAY_HEIGHT,FRAME_RATE,FLIP_METHOD);
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened())
    {
      printf("Failed to open camera.\n");
	    exit(-1);
    }
    cv::namedWindow("CSI Camera", cv::WINDOW_AUTOSIZE);

    while(1)
    {
        cv::Mat img, img_gray; 
        std::vector<cv::KeyPoint> keypoints;   
        if (!cap.read(img)) 
        {
            printf("Capture image error.");
            return;
        }
        // odometria
        cvtColor( img, img_gray, cv::COLOR_BayerBG2BGR  );
        cv::GaussianBlur(img_gray, img_gray, cv::Size(5,5), 0);

        fast->detect(img_gray, keypoints);
        cv::drawKeypoints(img_gray, keypoints, img, cv::Scalar(255,0,0));


        cv::imshow("CSI Camera",img);
        int keycode = cv::waitKey(10) & 0xff ; 
        if (keycode == 27) exit(-2) ;

    }



  return 0;
  
}
