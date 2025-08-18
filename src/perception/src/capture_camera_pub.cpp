#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

class CaptureCameraPub : public rclcpp::Node
{

public:

  CaptureCameraPub() : Node("publisher_capture_camera"){

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    cap_.open(0);
    camera_ready_ = false;
    if (!cap_.isOpened()){

      RCLCPP_ERROR(this->get_logger(), "Fail to open the camera!");
      return; 

    }
    camera_ready_ = true;

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CaptureCameraPub::publish_frame, this));

  }

  bool is_camera_ready() const{

    return camera_ready_;

  }
private:
  void publish_frame(){

    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()){

      RCLCPP_WARN(this->get_logger(), "The frame from capture_camera_pub is empty!");
      return;

    }

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_frame";

    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    publisher_->publish(*msg);

    RCLCPP_INFO(this->get_logger(), "I am publishing image.");


  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool camera_ready_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CaptureCameraPub>();
  if (!node->is_camera_ready()){

    rclcpp::shutdown();
    return 1;

  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
