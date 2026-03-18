//====================================================HEADERS AND GLOBALS======================================================================
#include <Eigen/Core>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/control/peripheral_actuators.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>

#include "std_msgs/msg/int32_multi_array.hpp" 
#include "keyboard_msgs/msg/key.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>

static const std::string kModeName = "Visual Follow Me";

float yaw_deg = 0;
float pitch_deg = 0;

int g_key ;
std::map<uint16_t, bool> key_states_;



//==============================================================================================================================================
























//============================================================== MODE CLASS CONSTRUCTOR ==================================================================
class VisualFollowMeMode : public px4_ros2::ModeBase
{
public:
  explicit VisualFollowMeMode(rclcpp::Node& node)
    : ModeBase(node, Settings{"VisualFollowMe"}.preventArming(false)) // Changed to false to allow testing
  {
          // {row_center(int), column_center(int), trackingActive(bool)}
    subscription_ = node.create_subscription<std_msgs::msg::Int32MultiArray>(
      "object_position", 10,
      std::bind(&VisualFollowMeMode::gimbal_control_callback, this, std::placeholders::_1));
    
    keydown_sub_ = node.create_subscription<keyboard_msgs::msg::Key>(
      "/keydown", 10, std::bind(&VisualFollowMeMode::update_key_down, this, std::placeholders::_1));

    // Subscriber for Key Release
    keyup_sub_ = node.create_subscription<keyboard_msgs::msg::Key>(
      "/keyup", 10, std::bind(&VisualFollowMeMode::update_key_up, this, std::placeholders::_1));

    // A loop to check our "variables" every 100ms
    timer_ = node.create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&VisualFollowMeMode::manual_commands, this));


    _vehicle_command_pub = node.create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

      VisualFollowMeMode::request_gimbal_control();

    _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  void onActivate() override { _target_set = false; }
  void onDeactivate() override {}

//======================================================== Drone  Control ======================================================================================
  void updateSetpoint(float dt_s) override
  { 
    if (!_vehicle_local_position->positionXYValid()) return;

    if (!_target_set && isTracking == 1) {
      _target_position = _vehicle_local_position->positionNed() + Eigen::Vector3f(10.f, 0.f, 0.f);
      _goto_setpoint->update(_target_position);
      _target_set = true;
    }

    
  }


//======================================================== Gimbal Control ======================================================================================

private:
//==================gimbal_control_callback==============================================================================================================================
  void gimbal_control_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
      if (msg->data.size() < 2) return;

      // Convert pixels to angles (Example: -45 to +45 degrees)
      isTracking = msg->data[2];
      if(isTracking == 1) { // 0 : notTracking
        yaw_deg = (static_cast<float>(msg->data[0]) / 1280.0f * 160.0f) - 80.0f;
        pitch_deg = 45 -  (static_cast<float>(msg->data[1]) / 720.0f * 90.0f);

        RCLCPP_INFO(node().get_logger(), "Tracking");
      }
      else {
        yaw_deg = 0;
        pitch_deg = 0;
      }

      px4_msgs::msg::VehicleCommand cmd{};
      cmd.timestamp = this->node().get_clock()->now().nanoseconds() / 1000;
      
      // Use the modern Gimbal Manager command
      cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW;
      cmd.param1 = pitch_deg; // PITCH
      cmd.param2 = yaw_deg;   // YAW
      cmd.param3 = 9;       // Pitch Rate (Ignore)
      cmd.param4 = 9;       // Yaw Rate (Ignore)
      cmd.param5 = 0.0f;      // Flags (Neutral frame)
      
      cmd.target_system = 1;
      cmd.target_component = 1; 
      cmd.source_system = 1;
      cmd.source_component = 1;
      cmd.from_external = true;

      _vehicle_command_pub->publish(cmd);
  }
//==================keyboard_callbacks==============================================================================================================================
  void update_key_up(const keyboard_msgs::msg::Key::SharedPtr msg) {
    g_key = msg->code;
    key_states_[msg->code] = 0;
  }
  void update_key_down(const keyboard_msgs::msg::Key::SharedPtr msg) {
    g_key = msg->code;
    key_states_[msg->code] = 1;
  }
//==================manual_commands==========================================================================================================
  void manual_commands() {
    if(key_states_['a']) {
      RCLCPP_INFO(node().get_logger(), "Key pressed : a ");
    }

    if(key_states_['a']) {
      RCLCPP_INFO(node().get_logger(), "Key released : a ");
    }
  }
//==================request_gimbal_control==============================================================================================================================

  void request_gimbal_control() {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = this->node().get_clock()->now().nanoseconds() / 1000;
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
    
    cmd.param1 = 1.0f;  // Sysid of the manager (usually 1)
    cmd.param2 = 1.0f;  // Compid of the manager (usually 1 or 154)
    cmd.param3 = -1.0f; // Primary control (Set to -1 to stay as is, or 1 to take over)
    cmd.param4 = -1.0f; // Secondary control
    
    // This is the most important part for some versions:
    // Some implementations use param7 or flags to force ownership.
    
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    _vehicle_command_pub->publish(cmd);
  } // private data type declaration

//==================PRIVATE VARIABLES==============================================================================================================================
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keydown_sub_;
    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keyup_sub_;

    std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;

    Eigen::Vector3f _target_position{0.f, 0.f, 0.f};
    bool _target_set{false};

    rclcpp::TimerBase::SharedPtr timer_;

    bool isTracking = 0;

};


using VisualFollowMeNode = px4_ros2::NodeWithMode<VisualFollowMeMode>;


// main ========================================================================================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::cout << "============== NOW GOING TO EXECUTE VISUAL FOLLOW ME NODE ===============" << std::endl;
  
  rclcpp::spin(std::make_shared<VisualFollowMeNode>("visual_follow_me", true));
  rclcpp::shutdown();
  return 0;
}

