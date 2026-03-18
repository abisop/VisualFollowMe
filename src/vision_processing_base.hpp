#pragma once

#include <opencv2/opencv.hpp>

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Subscribes to a ROS 2 image topic and stores the latest frame as an OpenCV matrix.
class VisionProcessingBase
{
public:
  explicit VisionProcessingBase(rclcpp::Node & node,
                                const std::string & image_topic = "/camera/image_raw");

  // Returns true if a frame has been received at least once.
  bool hasFrame() const;


  // Get a copy of the latest frame (empty if none yet).
  cv::Mat latestFrame() const;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Node & node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  mutable std::mutex frame_mutex_;
  cv::Mat last_frame_;
  bool has_frame_{false};

  cv::VideoWriter writer_;
};

