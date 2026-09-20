// motor_node -- the ONLY node that touches the motor.
//
// Subscribes to /motor/command (pre-deadband signed duty) and turns it into
// PWM + DIR + PS. Keeping the deadband feed-forward and the motor sign here
// means every producer speaks in plain command units, and changing the rig
// never touches the controller or the test nodes.
#include "me130_pendulum/hardware.hpp"
#include "me130_interfaces/msg/motor_command.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

class MotorNode : public rclcpp::Node
{
public:
    MotorNode() : rclcpp::Node("motor_node")
    {
        deadband_ = declare_parameter("deadband", 0.0);
        motor_sign_ = declare_parameter("motor_sign", 1.0);
        timeout_s_ = declare_parameter("command_timeout_s", 0.5);
        max_duty_ = declare_parameter("max_duty", 1.0);

        pwm_ = std::make_unique<me130::HardwarePwm>(me130::kPwmChannel, me130::kPwmFreqHz);
        gpio_ = std::make_unique<me130::MotorGpio>(me130::kGpioPs, me130::kGpioDir);
        drive(0.0);

        sub_ = create_subscription<me130_interfaces::msg::MotorCommand>(
            "/motor/command", rclcpp::SensorDataQoS(),
            [this](const me130_interfaces::msg::MotorCommand::SharedPtr msg) {
                last_command_ = now();
                drive(std::clamp(msg->u, -max_duty_, max_duty_));
            });

        // Watchdog: if whatever was driving us dies or is killed mid-test, the
        // motor must not keep running. Coasting is the safe state.
        watchdog_ = create_wall_timer(50ms, [this]() {
            if (!driving_) return;
            if ((now() - last_command_).seconds() > timeout_s_)
            {
                RCLCPP_WARN(get_logger(), "No command for %.2f s -- coasting.", timeout_s_);
                drive(0.0);
            }
        });

        RCLCPP_INFO(get_logger(), "motor_node up: deadband=%.4f motor_sign=%+.0f max_duty=%.2f",
                    deadband_, motor_sign_, max_duty_);
    }

    ~MotorNode() override
    {
        // Ordering matters: stop the PWM before the GPIO destructor drops PS.
        if (pwm_) pwm_->setDuty(0.0);
        if (gpio_) gpio_->coast();
    }

private:
    // Inverse of the deadband the motor actually has: a command of u produces
    // sign(u)*D + (1-D)*u duty, so the smallest nonzero command still moves.
    double applyDeadband(double u) const
    {
        if (std::fabs(u) < 1e-3) return 0.0;
        return std::copysign(deadband_, u) + (1.0 - deadband_) * u;
    }

    void drive(double u)
    {
        // Re-read every time: `ros2 param set` during a run should take effect.
        deadband_ = get_parameter("deadband").as_double();
        motor_sign_ = get_parameter("motor_sign").as_double();

        const double compensated = applyDeadband(std::clamp(u, -1.0, 1.0)) * motor_sign_;
        const double magnitude = std::fabs(compensated);

        if (magnitude < 1e-9)
        {
            pwm_->setDuty(0.0);
            gpio_->coast();
            driving_ = false;
            return;
        }

        // Drop the PWM before flipping direction so the bridge never reverses
        // while it is driving.
        pwm_->setDuty(0.0);
        gpio_->setDirection(compensated > 0.0 ? +1 : -1);
        gpio_->enableDriver();
        pwm_->setDuty(magnitude);
        driving_ = true;
    }

    double deadband_{0.0};
    double motor_sign_{1.0};
    double timeout_s_{0.5};
    double max_duty_{1.0};
    bool driving_{false};
    rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};

    std::unique_ptr<me130::HardwarePwm> pwm_;
    std::unique_ptr<me130::MotorGpio> gpio_;
    rclcpp::Subscription<me130_interfaces::msg::MotorCommand>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}
