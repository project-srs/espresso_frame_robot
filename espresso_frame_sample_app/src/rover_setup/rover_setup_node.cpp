#include "rover_setup_ros_if.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;  // NOLINT

class RoverSetupNode : public RoverSetupRosIf
{
public:
  RoverSetupNode() : RoverSetupRosIf() {}

protected:
  void onJoyReceived(const sensor_msgs::msg::Joy::SharedPtr msg) override
  {
    bool current_trigger_level = false;
    int target_index = getTriggerButtonIndex();
    if (0 <= target_index && target_index < (int)msg->buttons.size()) {
      current_trigger_level = (msg->buttons[target_index] != 0);
    }

    if (current_trigger_level && !last_trigger_state_) {
      RCLCPP_INFO(get_logger(), "trigger");
      arm_reqest_from_joy_ = true;
    }
    last_trigger_state_ = current_trigger_level;
  }

  void onMavrosStateReceived(const mavros_msgs::msg::State::SharedPtr msg) override
  {
    if (msg->connected) {
      if (mavros_connected_stamp_opt_) {
        float elapsed_duration = (now() - mavros_connected_stamp_opt_.value()).seconds();
        if (10.0f < elapsed_duration && !geo_origin_published_) {
          RCLCPP_INFO(get_logger(), "publish geo_origin");
          publishGeoOrigin();
          geo_origin_published_ = true;
        }
      } else {
        mavros_connected_stamp_opt_ = now();
      }
    }
  }

  void onTimer(void) override
  {
    // transit
    switch (next_state_) {
      case State::IDLE:
        if (arm_reqest_from_joy_) {
          next_state_ = State::SET_MODE;
          arm_reqest_from_joy_ = false;
        }
        break;
      case State::SET_MODE:
        next_state_ = State::WAIT_FOR_MODE;
        break;
      case State::WAIT_FOR_MODE:
        if (checkSetMode()) {
          next_state_ = State::DO_ARM;
          RCLCPP_INFO(get_logger(), "ok: set_mode");
        }
        break;
      case State::DO_ARM:
        next_state_ = State::WAIT_FOR_ARM;
        break;
      case State::WAIT_FOR_ARM:
        if (checkCommandArm()) {
          next_state_ = State::IDLE;
           RCLCPP_INFO(get_logger(), "ok: command_arm");
       }
        break;
      default:
        break;
    }

    // action
    switch (next_state_) {
      case State::SET_MODE:
        RCLCPP_INFO(get_logger(), "set_mode");
        callSetMode();
        break;
      case State::DO_ARM:
        RCLCPP_INFO(get_logger(), "command_arm");
        commandArm();
        break;
      case State::IDLE:
      case State::WAIT_FOR_MODE:
      case State::WAIT_FOR_ARM:
      default:
        break;
    }
  }

private:
  bool arm_reqest_from_joy_{false};
  std::optional<rclcpp::Time> mavros_connected_stamp_opt_{};
  bool geo_origin_published_{false};
  bool last_trigger_state_{false};

  enum class State {
    IDLE,
    SET_MODE,
    WAIT_FOR_MODE,
    DO_ARM,
    WAIT_FOR_ARM,
  } next_state_{State::IDLE};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto rover_setup = std::make_shared<RoverSetupNode>();
  rclcpp::spin(rover_setup);
  rclcpp::shutdown();
  return 0;
}
