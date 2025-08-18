#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraSubscriber : public rclcpp::Node
{

public:
    CameraSubscriber() : Node("camera_subscriber_node")
    {

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10,
                                                                           std::bind(&CameraSubscriber::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {

        try
        {

            cv::Mat frame;
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            if (frame.empty())
            {

                RCLCPP_WARN(this->get_logger(), "The frame in subscriber is empty");
                return;
            }

            cv::Mat img_rgb;
            cv::cvtColor(frame, img_rgb, cv::COLOR_BGR2RGB);

            cv::Mat img_resize;
            cv::resize(img_rgb, img_resize, cv::Size(640, 640));

            cv::Mat img_normalized;
            img_resize.convertTo(img_normalized, CV_32F, 1.0 / 255.0);

            // cv::imshow("test received", img_resize);
            // cv::waitKey(1); 

            RCLCPP_INFO(this->get_logger(), "Image processed: %dx%d", img_normalized.cols, img_normalized.rows);
        }
        catch (const cv_bridge::Exception &e)
        {

            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraSubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}