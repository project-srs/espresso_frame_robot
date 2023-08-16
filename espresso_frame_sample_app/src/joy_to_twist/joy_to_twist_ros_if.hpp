#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;  // NOLINT

struct AxisParameter
{
  int index{-1};
  float scale{1.0f};
};

struct JoyToTwistParameter
{
  std::string frame_id{"base_link"};
  AxisParameter linear_x{};
  AxisParameter linear_y{};
  AxisParameter linear_z{};
  AxisParameter angular_x{};
  AxisParameter angular_y{};
  AxisParameter angular_z{};
};

class JoyToTwistRosIf : public rclcpp::Node
{
public:
  JoyToTwistRosIf() : Node("joy_to_twist_node")
  {
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 1);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToTwistRosIf::onJoyReceived, this, std::placeholders::_1));

    // common parameters
    this->declare_parameter<std::string>("frame_id", "base_link");
    // axis parameters
    declrearAxisParameter("linear.x");
    declrearAxisParameter("linear.y");
    declrearAxisParameter("linear.z");
    declrearAxisParameter("angular.x");
    declrearAxisParameter("angular.y");
    declrearAxisParameter("angular.z");
  }

protected:
  void publishTwist(const geometry_msgs::msg::TwistStamped msg) { twist_pub_->publish(msg); }

  JoyToTwistParameter getParameter(void)
  {
    JoyToTwistParameter output;
    this->get_parameter("frame_id", output.frame_id);
    output.linear_x = getAxisParameter("linear.x");
    output.linear_y = getAxisParameter("linear.y");
    output.linear_z = getAxisParameter("linear.z");
    output.angular_x = getAxisParameter("angular.x");
    output.angular_y = getAxisParameter("angular.y");
    output.angular_z = getAxisParameter("angular.z");
    return output;
  }

  virtual void onJoyReceived(const sensor_msgs::msg::Joy::SharedPtr msg) = 0;

private:
  void declrearParameters(void)
  {
    // common parameters
    this->declare_parameter<std::string>("frame_id", "base_link");
    // axis parameters
    declrearAxisParameter("linear.x");
    declrearAxisParameter("linear.y");
    declrearAxisParameter("linear.z");
    declrearAxisParameter("angular.x");
    declrearAxisParameter("angular.y");
    declrearAxisParameter("angular.z");
  }

  AxisParameter declrearAxisParameter(const std::string & prefix)
  {
    AxisParameter output;
    this->declare_parameter<int>(prefix + ".index", -1);
    this->declare_parameter<float>(prefix + ".scale", 1.0f);
    return output;
  }

  AxisParameter getAxisParameter(const std::string & prefix)
  {
    AxisParameter output;
    this->get_parameter(prefix + ".index", output.index);
    this->get_parameter(prefix + ".scale", output.scale);
    return output;
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_{nullptr};
};
