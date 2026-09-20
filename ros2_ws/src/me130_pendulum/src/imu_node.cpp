// imu_node -- pendulum angle from the MPU6050.
//
// Runs the complementary filter and publishes theta / theta_dot. The zero is
// set through the "zero" service rather than a keystroke, so a launch file or
// a student script can do it: hanging straight down for the characterization
// tests, upright for balancing.
#include "me130_pendulum/hardware.hpp"
#include "me130_interfaces/msg/pendulum_angle.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cmath>
#include <memory>

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : rclcpp::Node("imu_node")
    {
        const double rate_hz = declare_parameter("publish_rate_hz", 500.0);
        const std::string device = declare_parameter("i2c_device", std::string("/dev/i2c-1"));
        const int address = declare_parameter("i2c_address", 0x68);
        const int cal_samples = declare_parameter("calibration_samples", 500);
        comp_ = declare_parameter("complementary_filter", 0.995);
        rate_sign_ = declare_parameter("rate_sign", 1.0);
        fail_max_ = declare_parameter("max_consecutive_read_failures", 8);
        // The characterization tests start with the rod hanging at rest, which
        // is exactly the pose the gyro calibration already requires. Take that
        // as theta = 0 so nobody has to remember to call the zero service --
        // and so an unzeroed log cannot quietly poison the identification.
        // Balancing leaves this false: there the zero is upright, set by hand.
        const bool zero_on_start = declare_parameter("zero_on_start", false);

        imu_ = std::make_unique<me130::Mpu6050>(device, address);

        RCLCPP_INFO(get_logger(), "Calibrating gyro bias -- keep the rod STILL (%d samples)...",
                    cal_samples);
        gyro_bias_ = imu_->calibrateGyroBiasRadPerSec(cal_samples);
        RCLCPP_INFO(get_logger(), "Gyro bias = %.6f rad/s", gyro_bias_);

        float ax = 0.0f, ay = 0.0f, gz = 0.0f;
        if (!imu_->read(ax, ay, gz))
            throw std::runtime_error("Initial MPU6050 read failed.");
        theta_ = std::atan2(static_cast<double>(ax), static_cast<double>(ay));
        if (zero_on_start)
        {
            theta_offset_ = theta_;
            theta_ = 0.0;
            RCLCPP_INFO(get_logger(),
                        "zero_on_start: taking the current pose (%.1f deg raw) as theta = 0",
                        theta_offset_ * 180.0 / M_PI);
        }

        pub_ = create_publisher<me130_interfaces::msg::PendulumAngle>(
            "/pendulum/angle", rclcpp::SensorDataQoS());

        zero_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/zero",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr res) {
                theta_offset_ += theta_;
                theta_ = 0.0;
                res->success = true;
                res->message = "theta zeroed at the current position";
                RCLCPP_INFO(get_logger(), "theta zeroed here");
            });

        last_ = now();
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz),
                                   [this]() { tick(); });

        RCLCPP_INFO(get_logger(), "imu_node up at %.0f Hz. Call the 'zero' service "
                                  "with the rod at its reference position.", rate_hz);
    }

private:
    void tick()
    {
        float ax = 0.0f, ay = 0.0f, gz = 0.0f;
        if (!imu_->read(ax, ay, gz))
        {
            if (++fails_ > fail_max_)
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                      "MPU6050 read failing (%d in a row). Consumers will "
                                      "see a stale angle -- check the I2C wiring.", fails_);
            return;   // publish nothing rather than a fabricated angle
        }
        fails_ = 0;

        const rclcpp::Time stamp = now();
        const double dt = (stamp - last_).seconds();
        last_ = stamp;
        if (dt <= 0.0 || dt > 0.5) return;   // first tick, or a stall: skip integrating

        comp_ = get_parameter("complementary_filter").as_double();
        rate_sign_ = get_parameter("rate_sign").as_double();

        const double rate = rate_sign_ * (static_cast<double>(gz) * M_PI / 180.0 - gyro_bias_);
        const double accel_angle =
            std::atan2(static_cast<double>(ax), static_cast<double>(ay)) - theta_offset_;

        theta_ = comp_ * (theta_ + rate * dt) + (1.0 - comp_) * accel_angle;

        me130_interfaces::msg::PendulumAngle msg;
        msg.header.stamp = stamp;
        msg.theta_rad = theta_;
        msg.theta_dot_rad_s = rate;
        pub_->publish(msg);
    }

    std::unique_ptr<me130::Mpu6050> imu_;
    rclcpp::Publisher<me130_interfaces::msg::PendulumAngle>::SharedPtr pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    double theta_{0.0};
    double theta_offset_{0.0};
    double gyro_bias_{0.0};
    double comp_{0.995};
    double rate_sign_{1.0};
    int fails_{0};
    int fail_max_{8};
    rclcpp::Time last_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
