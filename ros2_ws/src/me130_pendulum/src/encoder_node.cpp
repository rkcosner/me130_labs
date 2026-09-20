// encoder_node -- quadrature encoder on the motor shaft.
//
// Publishes raw counts and a differentiated rate. The motor characterization
// test needs this and nothing else, which is why it is its own block.
#include "me130_pendulum/hardware.hpp"
#include "me130_interfaces/msg/encoder_state.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

class EncoderNode : public rclcpp::Node
{
public:
    EncoderNode() : rclcpp::Node("encoder_node")
    {
        const double rate_hz = declare_parameter("publish_rate_hz", 200.0);

        encoder_ = std::make_unique<me130::QuadratureEncoder>(me130::kGpioEncA, me130::kGpioEncB);
        pub_ = create_publisher<me130_interfaces::msg::EncoderState>(
            "/encoder/state", rclcpp::SensorDataQoS());

        last_counts_ = encoder_->counts();
        last_time_ = now();

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz),
            [this]() { publish(); });

        RCLCPP_INFO(get_logger(), "encoder_node up: A=%d B=%d at %.0f Hz",
                    me130::kGpioEncA, me130::kGpioEncB, rate_hz);
    }

private:
    void publish()
    {
        const rclcpp::Time stamp = now();
        const long long counts = encoder_->counts();
        const double dt = (stamp - last_time_).seconds();

        me130_interfaces::msg::EncoderState msg;
        msg.header.stamp = stamp;
        msg.counts = counts;
        msg.counts_per_sec = (dt > 1e-9)
            ? static_cast<double>(counts - last_counts_) / dt
            : 0.0;

        last_counts_ = counts;
        last_time_ = stamp;
        pub_->publish(msg);
    }

    std::unique_ptr<me130::QuadratureEncoder> encoder_;
    rclcpp::Publisher<me130_interfaces::msg::EncoderState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    long long last_counts_{0};
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncoderNode>());
    rclcpp::shutdown();
    return 0;
}
