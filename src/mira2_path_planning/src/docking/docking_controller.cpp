#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/commands.hpp>
#include <custom_msgs/msg/telemetry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_utils/control_utils.hpp>


class DockingController : public rclcpp::Node {
    public:
    DockingController() : Node("docking_controller"), forwardPIDObj(), lateralPIDObj() {
        // Initialize PID gains
        depthPIDObj.kp = 5.5;    depthPIDObj.ki = 0.03;    depthPIDObj.kd = 31.5;  depthPIDObj.base_offset = 1500;
        forwardPIDObj.kp = -0.8;  forwardPIDObj.ki = -0.1;  forwardPIDObj.kd = -4.0;  forwardPIDObj.base_offset = 1500;
        lateralPIDObj.kp = -0.5;  lateralPIDObj.ki = -0.05; lateralPIDObj.kd = -2.0;  lateralPIDObj.base_offset = 1500;

        // Setup communications
        telemetrySub = this->create_subscription<custom_msgs::msg::Telemetry>(
            "/master/telemetry", 1, std::bind(&DockingController::telemetryCallback, this, std::placeholders::_1)
        )
        poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "dock_pose", 1, std::bind(&DockingController::poseCallback, this, std::placeholders::_1)
        )
        commandPub = this->create_publisher<custom_msgs::msg::Commands>("/master/commands", 1);

        // Create timer
        timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DockingController::mainLoop, this)
        )
    }

    private:
    // PID Controllers
    PID_Controller depthPIDObj;
    PID_Controller forwardPIDObj;
    PID_Controller lateralPIDObj;

    // Publishers & Subscribers
    rclcpp::Publisher<custom_msgs::msg::Commands>::SharedPtr commandPub;
    rclcpp::Subscription<custom_msgs::msg::Telemetry>:: SharedPtr telemetrySub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>:: SharedPtr poseSub;

    // Create message instance
    custom_msgs::msg::Commands cmdMsg;

    bool softwareArmFlag = false;

    // PID errors
    double currentDepth, depthSetpoint;
    double forwardSetpoint;
    double lateralSetpoint;
    double currentHeading;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time initTime;

    void telemetryCallback(const custom_msgs::msg::Telemetry::SharedPtr msg) {
        currentHeading = msg->heading;
        currentDepth = msg->external_pressure;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        forwardSetpoint = msg->pose.position.x;
        lateralSetpoint = msg->pose.position.y;
        depthSetpoint = msg->pose.position.z;
    }

    void mainLoop() {
        if (!softwareArmFlag) {
            softwareArmFlag = true;
            initTime = this->now();
        }

        cmdMsg.mode = "ALT_HOLD";
        rclcpp::Time timeNow = this->now();
        cmdMsg.arm = true;

        if (softwareArmFlag) {
            float pidDepth = depthPIDObj.pid_control(
                depthSetpoint, (time_now - initTime).seconds(), false
            );

            float pidForward = forwardPIDObj.pid_control(
                forwardSetpoint, (timeNow - initTime).seconds(), false
            );

            float pidLateral = lateralPIDObj.pid_control(
                lateralSetpoint, (timeNow - initTime).seconds(), false
            );

            cmdMsg.thrust = pidDepth;
            cmdMsg.forward = pidForward;
            cmdMsg.lateral = pidLateral;
            RCLCPP_INFO(this->get_logger(), "Approaching Dock XY");
        } else {
            forwardPIDObj.emptyError();
            lateralPIDObj.emptyError();
            cmdMsg.arm = false;
            cmdMsg.mode = "ALT_HOLD";
            cmdMsg.thrust = 1500;
            cmdMsg.forward = 1500;
            cmdMsg.lateral = 1500;
        }

        commandPub->publish(cmdMsg);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DockingController>());
    rclcpp::shutdown();
    return 0;
}