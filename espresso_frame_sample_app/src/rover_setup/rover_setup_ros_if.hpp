#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;  // NOLINT

class RoverSetupRosIf : public rclcpp::Node
{
public:
  RoverSetupRosIf() : Node("rover_setup_node")
  {
    geo_origin_pub_ = this->create_publisher<geographic_msgs::msg::GeoPointStamped>(
      "/device/mavros/global_position/set_gp_origin", 1);
    mode_client_ = create_client<mavros_msgs::srv::SetMode>("/device/mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/device/mavros/cmd/arming");
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&RoverSetupRosIf::onJoyReceived, this, std::placeholders::_1));
    mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/device/mavros/state", 10,
      std::bind(&RoverSetupRosIf::onMavrosStateReceived, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(500ms, std::bind(&RoverSetupRosIf::onTimer, this));

    this->declare_parameter<int>("trigger_button.index", -1);
  }

  int getTriggerButtonIndex(void)
  {
    int value = 0;
    this->get_parameter("trigger_button.index", value);
    return value;
  }

  void publishGeoOrigin(void)
  {
    geographic_msgs::msg::GeoPointStamped geo_point{};
    geo_origin_pub_->publish(geo_point);
  }

  void callSetMode(void)
  {
    if (!mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "mode_request not connect\n");
      set_mode_done_ = true;
      return;
    }

    auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    mode_request->custom_mode = "GUIDED";
    auto mode_future = mode_client_->async_send_request(
      mode_request, std::bind(&RoverSetupRosIf::onSetMode, this, std::placeholders::_1));
  }

  bool checkSetMode(void)
  {
    bool result = set_mode_done_;
    set_mode_done_ = false;
    return result;
  }

  void commandArm(void)
  {
    if (!arm_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "arm_request not connect");
      cmd_arm_done_ = true;
      return;
    }

    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;
    auto arm_future = arm_client_->async_send_request(
      arm_request, std::bind(&RoverSetupRosIf::onArm, this, std::placeholders::_1));
  }

  bool checkCommandArm(void)
  {
    bool result = cmd_arm_done_;
    cmd_arm_done_ = false;
    return result;
  }

protected:
  virtual void onTimer(void) = 0;
  virtual void onJoyReceived(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;
  virtual void onMavrosStateReceived(const mavros_msgs::msg::State::SharedPtr msg) = 0;

private:
  void onSetMode(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
  {
    (void)future;
    set_mode_done_ = true;
  }

  void onArm(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
  {
    (void)future;
    cmd_arm_done_ = true;
  }

  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr geo_origin_pub_{nullptr};
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_{nullptr};
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_{nullptr};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  bool set_mode_done_{false};
  bool cmd_arm_done_{false};
};
