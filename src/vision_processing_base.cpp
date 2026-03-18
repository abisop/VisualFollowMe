#include "vision_processing_base.hpp"
#include <opencv2/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp" // Header for arrays


cv::Rect roi;
bool drawing = false;

bool trackingActive = false;
cv::Ptr<cv::Tracker> tracker;

bool trackerInitialized = false;

rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr _object_position_publisher_;

// helper functions
//=============================== mouse callback function ========================================================
//=============================== has frame function      ========================================================
bool VisionProcessingBase::hasFrame() const
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  return has_frame_;
}

//=============================== latest frame function   ========================================================
cv::Mat VisionProcessingBase::latestFrame() const
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  return last_frame_.clone();
}

void onMouse(int event, int x, int y, int flags, void* userdata) {
  if (event == cv::EVENT_LBUTTONDOWN) {
      drawing = true;
      roi = cv::Rect(x, y, 0, 0);
      trackingActive = false;
  } 
  else if (event == cv::EVENT_MOUSEMOVE) {
      if (drawing) {
          roi.width = x - roi.x;
          roi.height = y - roi.y;
      }
  } 
  else if (event == cv::EVENT_LBUTTONUP) {
      drawing = false;
      // Handle negative width/height if dragged backwards
      if (roi.width < 0) { roi.x += roi.width; roi.width *= -1; }
      if (roi.height < 0) { roi.y += roi.height; roi.height *= -1; }

      if (roi.width > 5 && roi.height > 5) {
        trackingActive = true;
        // Initialize the tracker with the selected ROI
        tracker = cv::TrackerCSRT::create(); 
        // We'll initialize it in the main loop to get the latest frame
    }
  }
}

//================================ constructor            ========================================================
VisionProcessingBase::VisionProcessingBase(rclcpp::Node & node,
                                           const std::string & image_topic)
  : node_(node)
{
  image_sub_ = node_.create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(),
    std::bind(&VisionProcessingBase::imageCallback, this, std::placeholders::_1));

  cv::namedWindow("Live Feed");
  
  _object_position_publisher_ = node_.create_publisher<std_msgs::msg::Int32MultiArray>("object_position", 10);


  cv::setMouseCallback("Live Feed", onMouse);
  RCLCPP_INFO(node_.get_logger(),
              "VisionProcessingBase subscribing to image topic: %s", image_topic.c_str());
}


//=============================== image callback function ========================================================
void VisionProcessingBase::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat current_frame = cv_ptr->image.clone();

    int frame_width = current_frame.cols;
        int frame_height = current_frame.rows;
        double fps = 30.0; 

    std::string pipeline = "appsrc ! videoconvert ! video/x-raw, format=I420 ! x264enc tune=zerolatency bitrate=500 ! "
                            "rtph264pay config-interval=1 pt=96 ! "
                            "udpsink host=127.0.0.1 port=5600";

    writer_.open(pipeline, cv::CAP_GSTREAMER, 0, fps, cv::Size(frame_width, frame_height));


    if (trackingActive) {
      if (!trackerInitialized) {
          // First time setup for this specific selection
          tracker->init(current_frame, roi);
          trackerInitialized = true;
      }

      // Update the tracker's position in the new frame
      if (tracker->update(current_frame, roi)) {
          cv::rectangle(current_frame, roi, cv::Scalar(0, 255, 0), 2);
          cv::putText(current_frame, "Tracking", cv::Point(10, 30), 
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
      } else {
          cv::putText(current_frame, "Lost!", cv::Point(10, 30), 
                      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
      }
    }

    auto obj_pos = std_msgs::msg::Int32MultiArray();

    std::vector<int32_t> obj_pos_raw = {roi.x + roi.width/2, roi.y + roi.height/2, trackingActive};

    obj_pos.data = obj_pos_raw;

    _object_position_publisher_->publish(obj_pos);

    

    if (drawing) {
      cv::rectangle(current_frame, roi, cv::Scalar(255, 0, 0), 2);
      trackerInitialized = false; // Prepare for new initialization
    }  

    writer_.write(current_frame);
    cv::imshow("Live Feed", current_frame);
    cv::waitKey(1);
    

  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "cv_bridge exception: %s", e.what());
  }
}

//=============================== vision tracking function ========================================================
std::vector<int> vision_tracking(cv::Mat croppedFrame) {
  std::vector<int>coordinates = {0,0};
  return coordinates; 
}

//=============================== main function ========================================================
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::cout << "EXECUTING - VISION PROCESSING BASE NODE" << std::endl;
  
  auto vision_node = std::make_shared<rclcpp::Node>("vision_processing");
  VisionProcessingBase vision(*vision_node, "/camera/image_raw");

  rclcpp::executors::MultiThreadedExecutor executor;
  
  executor.add_node(vision_node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}