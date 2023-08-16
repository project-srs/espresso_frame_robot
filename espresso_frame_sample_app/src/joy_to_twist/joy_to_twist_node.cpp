#include "joy_to_twist_ros_if.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;  // NOLINT

class JoyToCmdRate : public JoyToTwistRosIf
{
public:
  JoyToCmdRate() : JoyToTwistRosIf()
  {
  }

protected:
  void onJoyReceived(const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    JoyToTwistParameter param = getParameter();
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = msg->header.stamp;
    twist.header.frame_id = param.frame_id;
    twist.twist.linear.x = getAxisValue(*msg, param.linear_x);
    twist.twist.linear.y = getAxisValue(*msg, param.linear_y);
    twist.twist.linear.z = getAxisValue(*msg, param.linear_z);
    twist.twist.angular.x = getAxisValue(*msg, param.angular_x);
    twist.twist.angular.y = getAxisValue(*msg, param.angular_y);
    twist.twist.angular.z = getAxisValue(*msg, param.angular_z);
    publishTwist(twist);
  }

private:
  float getAxisValue(const sensor_msgs::msg::Joy joy, const AxisParameter param) {
    if (0 <= param.index && param.index < (int)joy.axes.size()) {
      return param.scale * joy.axes[param.index];
    } 
    return 0.0f;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto joy_to_cmd_rate = std::make_shared<JoyToCmdRate>();
  rclcpp::spin(joy_to_cmd_rate);
  rclcpp::shutdown();
  return 0;
}
