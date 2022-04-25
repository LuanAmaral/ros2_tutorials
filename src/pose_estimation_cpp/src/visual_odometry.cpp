#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.h>

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

std::string pipeline = gstreamer_pipeline(CAPTURE_WIDTH,CAPTURE_HEIGHT,DISPLAY_WIDTH,DISPLAY_HEIGHT,FRAME_RATE,FLIP_METHOD);
cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(): Node("minimal_publisher"), count_(0)
  {
    geometry_msgs::msg::Pos
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("Pose", 10);
    timer_=this->create_wall_timer(1/FRAME_RATE,std::bind(&MinimalPublisher::timer_callback, this));

    if(!cap.isOpened())
    {
      std::cout<<"Failed to open camera."<<std::endl;
	    exit(-1);
    }
    cv::namedWindow("CSI Camera", cv::WINDOW_AUTOSIZE);
  }

private:
  void visual_odometry()
  {
    cv::Mat img;    if (!cap.read(img)) 
    {
      printf("Capture image error.");
      return;
    }
    // odometria

    cv::imshow("CSI Camera",img);
    int keycode = cv::waitKey(10) & 0xff ; 
    if (keycode == 27) exit(-2) ;
  }

  void timer_callback()
  {
    visual_odometry();
    RCLCPP_INFO(this->get_logger(), "Publishing");
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
  
}
