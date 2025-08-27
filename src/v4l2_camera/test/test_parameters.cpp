#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "v4l2_camera/parameters.hpp"

TEST(ParametersTest, RejectInvalidOutputEncoding) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  v4l2_camera::Parameters params(
    node->get_node_parameters_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface());
  params.declareStaticParameters();
  params.declareOutputParameters();
  params.setParameterChangedCallback([](rclcpp::Parameter){});
  auto result = node->set_parameter(rclcpp::Parameter("output_encoding", "invalid"));
  EXPECT_FALSE(result.successful);
  rclcpp::shutdown();
}

TEST(ParametersTest, RejectInvalidImageSize) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node2");
  node->declare_parameter<std::vector<int64_t>>("image_size", {640, 480});
  v4l2_camera::Parameters params(
    node->get_node_parameters_interface(),
    node->get_node_topics_interface(),
    node->get_node_logging_interface());
  params.setParameterChangedCallback([](rclcpp::Parameter){});
  auto result = node->set_parameter(rclcpp::Parameter("image_size", std::vector<int64_t>{-1, 480}));
  EXPECT_FALSE(result.successful);
  rclcpp::shutdown();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
