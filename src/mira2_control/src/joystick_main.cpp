#include <algorithm>
#include <cstddef>
#include <cmath>
#include <stdexcept>

#include <custom_msgs/msg/commands_normalized.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <utils/parameters.hpp>

namespace {

// Standard Xbox/SDL joystick mapping published by joy_node.
constexpr std::size_t kLeftStickX = 0;
constexpr std::size_t kLeftStickY = 1;
constexpr std::size_t kLeftTrigger = 2;
constexpr std::size_t kRightStickX = 3;
constexpr std::size_t kRightStickY = 4;
constexpr std::size_t kRightTrigger = 5;

constexpr std::size_t kButtonA = 0;
constexpr std::size_t kButtonB = 1;
constexpr std::size_t kButtonX = 2;
constexpr std::size_t kButtonY = 3;
constexpr std::size_t kButtonLeftBumper = 4;
constexpr std::size_t kButtonRightBumper = 5;

constexpr std::size_t kRequiredAxes = 6;
constexpr std::size_t kRequiredButtons = 6;

}  // namespace

class JoystickController : public rclcpp::Node {
public:
  JoystickController() : Node("joystick_controller") {
    sensitivity_ = utils::parameter(*this, "sensitivity", 0.6);
    yaw_sensitivity_ = utils::parameter(*this, "yaw_sensitivity", 0.5);
    pitch_sensitivity_ = utils::parameter(*this, "pitch_sensitivity", 0.5);
    deadzone_ = utils::parameter(*this, "deadzone", 0.1);
    qgc_layout_ = utils::parameter(*this, "qgc_layout", false);

    if (!std::isfinite(deadzone_) || deadzone_ < 0 || deadzone_ >= 1 ||
        !std::isfinite(sensitivity_) || sensitivity_ < 0 || sensitivity_ > 1 ||
        !std::isfinite(yaw_sensitivity_) || yaw_sensitivity_ < 0 || yaw_sensitivity_ > 1 ||
        !std::isfinite(pitch_sensitivity_) || pitch_sensitivity_ < 0 || pitch_sensitivity_ > 1) {
      throw std::invalid_argument("Joystick sensitivity/deadzone outside normalized range");
    }

    publisher_ = create_publisher<custom_msgs::msg::CommandsNormalized>(
        "mira/command", 1);
    subscription_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, [this](sensor_msgs::msg::Joy::SharedPtr joy) { callback(*joy); });
    command_.mode = "STABILIZE";
    print_layout();
  }

private:
  float effort(float value, double sensitivity) const {
    // Ignore small stick noise, then scale the remaining normalized input and
    // clamp it to the CommandsNormalized range of [-1, 1].
    return std::abs(value) < deadzone_
               ? 0.0f
               : std::max(-1.0f, std::min(1.0f, static_cast<float>(value * sensitivity)));
  }

  void print_layout() const {
    const char *vertical_control = qgc_layout_ ? "thrust (up = ascend)" : "pitch";
    const char *trigger_control = qgc_layout_ ? "pitch (RT = up, LT = down)"
                                              : "thrust (LT = up, RT = down)";
    RCLCPP_INFO(get_logger(), "Joystick layout: standard Xbox/SDL mapping (%s layout)",
                qgc_layout_ ? "QGC" : "default");
    RCLCPP_INFO(get_logger(), "  Left stick: forward/back and lateral; right stick: yaw and %s",
                vertical_control);
    RCLCPP_INFO(get_logger(), "  Triggers: %s", trigger_control);
    RCLCPP_INFO(get_logger(), "  A: arm/disarm; B: MANUAL; X: ALT_HOLD; Y: STABILIZE");
    RCLCPP_INFO(get_logger(), "  LB: roll left; RB: roll right");
  }

  void publish_neutral_command() {
    command_ = custom_msgs::msg::CommandsNormalized();
    command_.header.stamp = now();
    command_.header.frame_id = "base_link";
    command_.mode = "STABILIZE";
    publisher_->publish(command_);
  }

  void callback(const sensor_msgs::msg::Joy &joy) {
    if (joy.axes.size() < kRequiredAxes || joy.buttons.size() < kRequiredButtons ||
        !std::all_of(joy.axes.begin(), joy.axes.end(),
                     [](float value) { return std::isfinite(value); })) {
      publish_neutral_command();
      return;
    }

    if (joy.buttons[kButtonA] && !previous_arm_button_) command_.arm = !command_.arm;
    previous_arm_button_ = joy.buttons[kButtonA];
    if (joy.buttons[kButtonB]) command_.mode = "MANUAL";
    if (joy.buttons[kButtonX]) command_.mode = "ALT_HOLD";
    if (joy.buttons[kButtonY]) command_.mode = "STABILIZE";

    command_.forward = effort(joy.axes[kLeftStickY], sensitivity_);
    command_.lateral = effort(joy.axes[kLeftStickX], sensitivity_);  // positive = left
    command_.yaw = effort(joy.axes[kRightStickX], yaw_sensitivity_); // positive = CCW

    // Triggers idle at +1 and reach -1 when fully pressed.  Mapping [-1, +1]
    // to [1, 0] gives each trigger a positive amount of upward/downward input.
    const float up = (1.0f - joy.axes[kRightTrigger]) / 2.0f;
    const float down = (1.0f - joy.axes[kLeftTrigger]) / 2.0f;

    // Subtracting down from up makes opposing trigger presses cancel out.
    // Default layout uses triggers for thrust and inverts right-stick Y for
    // pitch because this controller reports up as a negative stick value.
    command_.thrust = qgc_layout_ ? effort(joy.axes[kRightStickY], sensitivity_)
                                  : effort(up - down, sensitivity_);
    command_.pitch = qgc_layout_ ? effort(up - down, pitch_sensitivity_)
                                 : effort(-joy.axes[kRightStickY], pitch_sensitivity_);
    command_.roll = joy.buttons[kButtonLeftBumper]
                        ? -0.25f
                        : (joy.buttons[kButtonRightBumper] ? 0.25f : 0.0f);
    command_.header.stamp = now();
    command_.header.frame_id = "base_link";
    publisher_->publish(command_);
  }

  bool qgc_layout_;
  bool previous_arm_button_ = false;
  double sensitivity_;
  double yaw_sensitivity_;
  double pitch_sensitivity_;
  double deadzone_;
  custom_msgs::msg::CommandsNormalized command_;
  rclcpp::Publisher<custom_msgs::msg::CommandsNormalized>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoystickController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
