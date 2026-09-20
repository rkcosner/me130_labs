// step_sequence_node -- drives the open-loop step test, hands-off.
//
// Rod HANGING DOWN. Each duty is held for hold_s, then the motor coasts for
// rest_s. Every segment therefore contains a FORCED response followed by a
// FREE response, which is what step_response.py separates on the mode column.
//
// Signs alternate so the rod stays near its rest position instead of winding
// one way. The node exits when the sequence finishes; the launch file turns
// that into a clean shutdown of the whole stack.
#include "me130_interfaces/msg/motor_command.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

class StepSequenceNode : public rclcpp::Node
{
public:
    StepSequenceNode() : rclcpp::Node("step_sequence_node")
    {
        duties_ = declare_parameter("step_duties",
                                    std::vector<double>{0.10, -0.10, 0.15, -0.15,
                                                        0.20, -0.20, 0.25, -0.25});
        hold_s_ = declare_parameter("hold_s", 1.0);
        rest_s_ = declare_parameter("rest_s", 2.5);
        const double rate_hz = declare_parameter("publish_rate_hz", 500.0);
        // DDS discovery is NOT instant -- measured at ~2.5 s on this Pi. A
        // fixed delay is a guess; instead wait until the expected consumers
        // have actually connected to this publisher, then settle briefly.
        // Without this the logger misses the whole first segment, which then
        // reads as a short malformed step (or vanishes from the analysis).
        //   1 = motor_node only, 2 = motor_node + logger_node.
        required_subs_ = declare_parameter("wait_for_subscribers", 1);
        wait_timeout_s_ = declare_parameter("wait_timeout_s", 15.0);
        startup_s_ = declare_parameter("startup_delay_s", 0.5);

        if (duties_.empty())
        {
            RCLCPP_ERROR(get_logger(), "step_duties is empty -- nothing to run.");
            rclcpp::shutdown();
            return;
        }

        pub_ = create_publisher<me130_interfaces::msg::MotorCommand>(
            "/motor/command", rclcpp::SensorDataQoS());

        dt_ = 1.0 / rate_hz;
        timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this]() { tick(); });

        RCLCPP_INFO(get_logger(), "step sequence: %zu steps, %.2f s hold / %.2f s rest "
                                  "(~%.0f s total). Rod should be HANGING DOWN.",
                    duties_.size(), hold_s_, rest_s_,
                    duties_.size() * (hold_s_ + rest_s_));
        RCLCPP_INFO(get_logger(), "step 1/%zu  duty=%+.3f", duties_.size(), duties_[0]);
    }

private:
    void publish(double u, const char *mode, double step_duty)
    {
        me130_interfaces::msg::MotorCommand msg;
        msg.header.stamp = now();
        msg.u = u;
        msg.mode = mode;
        msg.step_duty = step_duty;
        msg.segment = index_ + 1;
        pub_->publish(msg);
    }

    void tick()
    {
        if (!started_)
        {
            startup_elapsed_ += dt_;
            publish(0.0, "coast", 0.0);

            const int subs = static_cast<int>(pub_->get_subscription_count());
            if (!connected_ && subs >= required_subs_)
            {
                connected_ = true;
                settle_at_ = startup_elapsed_ + startup_s_;
                RCLCPP_INFO(get_logger(), "%d subscriber(s) connected after %.2f s",
                            subs, startup_elapsed_);
            }
            if (!connected_)
            {
                if (startup_elapsed_ >= wait_timeout_s_)
                {
                    RCLCPP_WARN(get_logger(),
                                "Only %d of %d subscriber(s) after %.1f s -- starting anyway. "
                                "Any log from this run may be missing its first segment.",
                                subs, required_subs_, startup_elapsed_);
                    started_ = true;
                }
                return;
            }
            if (startup_elapsed_ < settle_at_) return;
            started_ = true;
        }
        elapsed_ += dt_;

        if (holding_)
        {
            if (elapsed_ >= hold_s_) { holding_ = false; elapsed_ = 0.0; }
            else { publish(duties_[index_], "step", duties_[index_]); return; }
        }

        if (elapsed_ < rest_s_) { publish(0.0, "coast", 0.0); return; }

        if (++index_ >= duties_.size())
        {
            publish(0.0, "coast", 0.0);
            RCLCPP_INFO(get_logger(), "step sequence COMPLETE");
            timer_->cancel();
            rclcpp::shutdown();
            return;
        }

        holding_ = true;
        elapsed_ = 0.0;
        RCLCPP_INFO(get_logger(), "step %zu/%zu  duty=%+.3f",
                    index_ + 1, duties_.size(), duties_[index_]);
        publish(duties_[index_], "step", duties_[index_]);
    }

    std::vector<double> duties_;
    double hold_s_{1.0}, rest_s_{2.5}, dt_{0.002}, elapsed_{0.0};
    double startup_s_{0.5}, startup_elapsed_{0.0}, settle_at_{0.0}, wait_timeout_s_{15.0};
    int required_subs_{1};
    bool connected_{false}, started_{false};
    std::size_t index_{0};
    bool holding_{true};
    rclcpp::Publisher<me130_interfaces::msg::MotorCommand>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepSequenceNode>());
    rclcpp::shutdown();
    return 0;
}
