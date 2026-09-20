// controller_node -- the block students edit.
//
// Reads /pendulum/angle, writes /motor/command. It knows nothing about the
// motor driver, the deadband or the wiring; motor_node owns all of that.
//
// The gains default to ZERO, so a fresh checkout is inert: arming it does
// nothing until the student sets kp/kd/ki. That is deliberate -- a pendulum
// that lunges on the first launch is how hardware gets broken.
#include "me130_interfaces/msg/motor_command.hpp"
#include "me130_interfaces/msg/pendulum_angle.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode() : rclcpp::Node("controller_node")
    {
        // Students set these. Zero until they do.
        declare_parameter("kp", 0.0);
        declare_parameter("ki", 0.0);
        declare_parameter("kd", 0.0);

        integral_limit_ = declare_parameter("integral_limit", 0.5);
        theta_max_deg_ = declare_parameter("theta_max_deg", 90.0);
        arm_window_deg_ = declare_parameter("arm_window_deg", 5.0);
        auto_arm_ = declare_parameter("auto_arm", false);

        pub_ = create_publisher<me130_interfaces::msg::MotorCommand>(
            "/motor/command", rclcpp::SensorDataQoS());

        sub_ = create_subscription<me130_interfaces::msg::PendulumAngle>(
            "/pendulum/angle", rclcpp::SensorDataQoS(),
            [this](const me130_interfaces::msg::PendulumAngle::SharedPtr msg) { onAngle(msg); });

        arm_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/arm",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                if (std::fabs(theta_) > arm_window_deg_ * M_PI / 180.0)
                {
                    res->success = false;
                    res->message = "too far from the zero to arm";
                    RCLCPP_WARN(get_logger(), "Arm refused: |theta| = %.1f deg exceeds the "
                                              "%.1f deg window.",
                                theta_ * 180.0 / M_PI, arm_window_deg_);
                    return;
                }
                integral_ = 0.0;
                armed_ = true;
                res->success = true;
                res->message = "armed";
                RCLCPP_INFO(get_logger(), "ARMED (kp=%.3f ki=%.3f kd=%.3f)",
                            get_parameter("kp").as_double(),
                            get_parameter("ki").as_double(),
                            get_parameter("kd").as_double());
            });

        disarm_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/disarm",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                disarm("service request");
                res->success = true;
                res->message = "disarmed";
            });

        RCLCPP_INFO(get_logger(),
                    "controller_node up, DISARMED. Gains are zero until you set them:\n"
                    "  ros2 param set /controller_node kp 10.0\n"
                    "  ros2 service call /controller_node/arm std_srvs/srv/Trigger");
    }

private:
    void disarm(const char *why)
    {
        if (armed_) RCLCPP_WARN(get_logger(), "DISARMED (%s)", why);
        armed_ = false;
        integral_ = 0.0;
        publish(0.0, "coast");
    }

    void publish(double u, const char *mode)
    {
        me130_interfaces::msg::MotorCommand msg;
        msg.header.stamp = now();
        msg.u = u;
        msg.mode = mode;
        msg.segment = segment_;
        pub_->publish(msg);
    }

    void onAngle(const me130_interfaces::msg::PendulumAngle::SharedPtr msg)
    {
        const rclcpp::Time stamp = msg->header.stamp;
        const double dt = last_valid_ ? (stamp - last_).seconds() : 0.0;
        last_ = stamp;
        last_valid_ = true;

        theta_ = msg->theta_rad;
        const double theta_dot = msg->theta_dot_rad_s;

        if (std::fabs(theta_) > theta_max_deg_ * M_PI / 180.0)
        {
            disarm("past theta_max -- fell over");
            return;
        }

        if (!armed_)
        {
            if (auto_arm_ && std::fabs(theta_) < arm_window_deg_ * M_PI / 180.0)
            {
                armed_ = true;
                integral_ = 0.0;
                RCLCPP_INFO(get_logger(), "auto-armed inside the arm window");
            }
            else
            {
                publish(0.0, "coast");
                return;
            }
        }

        const double kp = get_parameter("kp").as_double();
        const double ki = get_parameter("ki").as_double();
        const double kd = get_parameter("kd").as_double();

        double integral_term = 0.0;
        if (std::fabs(ki) > 1e-9 && dt > 0.0 && dt < 0.5)
        {
            integral_ += theta_ * dt;
            const double bound = integral_limit_ / std::fabs(ki);
            integral_ = std::clamp(integral_, -bound, bound);
            integral_term = ki * integral_;
        }

        const double u = -(kp * theta_ + integral_term + kd * theta_dot);
        publish(std::clamp(u, -1.0, 1.0), "pd");
    }

    rclcpp::Publisher<me130_interfaces::msg::MotorCommand>::SharedPtr pub_;
    rclcpp::Subscription<me130_interfaces::msg::PendulumAngle>::SharedPtr sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_, disarm_srv_;

    bool armed_{false};
    bool auto_arm_{false};
    bool last_valid_{false};
    double theta_{0.0};
    double integral_{0.0};
    double integral_limit_{0.5};
    double theta_max_deg_{90.0};
    double arm_window_deg_{5.0};
    int segment_{1};
    rclcpp::Time last_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
