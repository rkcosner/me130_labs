// logger_node -- writes the CSV the Python analysis scripts already read.
//
// A row is emitted for every /pendulum/angle, carrying the most recent motor
// command alongside it.
//
// Logging on the ANGLE topic rather than the command topic matters. theta is
// the signal the analysis differentiates twice, so its timestamp has to be
// exact; pairing it with a command timestamp from a different node leaves up
// to a tick of skew, and that skew biases the identified a and c by over 10%.
// The command is an input that only changes at segment boundaries, so a tick
// of lag on u is harmless by comparison.
//
// The column layout is unchanged from the pre-ROS firmware, so
// step_response.py and freq_response.py work without modification.
#include "me130_interfaces/msg/encoder_state.hpp"
#include "me130_interfaces/msg/motor_command.hpp"
#include "me130_interfaces/msg/pendulum_angle.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

class LoggerNode : public rclcpp::Node
{
public:
    LoggerNode() : rclcpp::Node("logger_node")
    {
        const std::string prefix = declare_parameter("prefix", std::string("full"));
        const std::string dir = declare_parameter("output_dir", std::string("."));

        std::time_t t = std::time(nullptr);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
        path_ = dir + "/" + prefix + "_" + stamp + ".csv";

        // A big stream buffer, installed before open(): a 500 Hz writer should
        // not hit a syscall per row. Rows end in '\n', never std::endl.
        buffer_.assign(1 << 20, '\0');
        file_.rdbuf()->pubsetbuf(buffer_.data(), static_cast<std::streamsize>(buffer_.size()));
        file_.open(path_);
        if (!file_)
        {
            RCLCPP_FATAL(get_logger(), "Could not open %s", path_.c_str());
            throw std::runtime_error("Could not open the log file.");
        }
        // Default precision would quantize t_s past ~100 s.
        file_ << std::fixed << std::setprecision(6);
        file_ << "t_s,mode,u_cmd,theta_rad,theta_dot_rad_s,enc,freq_hz,amp,segment,step_duty\n";

        angle_sub_ = create_subscription<me130_interfaces::msg::PendulumAngle>(
            "/pendulum/angle", rclcpp::SensorDataQoS(),
            [this](const me130_interfaces::msg::PendulumAngle::SharedPtr m) { write(m); });

        encoder_sub_ = create_subscription<me130_interfaces::msg::EncoderState>(
            "/encoder/state", rclcpp::SensorDataQoS(),
            [this](const me130_interfaces::msg::EncoderState::SharedPtr m) {
                counts_ = m->counts;
            });

        command_sub_ = create_subscription<me130_interfaces::msg::MotorCommand>(
            "/motor/command", rclcpp::SensorDataQoS(),
            [this](const me130_interfaces::msg::MotorCommand::SharedPtr m) {
                have_command_ = true;
                command_ = *m;
            });

        RCLCPP_INFO(get_logger(), "logging to %s", path_.c_str());
    }

    ~LoggerNode() override
    {
        if (file_.is_open())
        {
            file_.flush();
            file_.close();
            RCLCPP_INFO(get_logger(), "wrote %ld rows to %s", rows_, path_.c_str());
        }
    }

private:
    // Called on every angle sample: theta and t_s are then exactly consistent.
    void write(const me130_interfaces::msg::PendulumAngle::SharedPtr &m)
    {
        // Nothing to log until something is driving the motor.
        if (!have_command_) return;

        const rclcpp::Time stamp = m->header.stamp;
        if (rows_ == 0) start_ = stamp;

        file_ << (stamp - start_).seconds() << ',' << command_.mode << ','
              << command_.u << ',' << m->theta_rad << ',' << m->theta_dot_rad_s << ','
              << counts_ << ',' << command_.freq_hz << ',' << command_.amplitude << ','
              << command_.segment << ',' << command_.step_duty << '\n';
        ++rows_;
    }

    std::ofstream file_;
    std::vector<char> buffer_;
    std::string path_;
    long rows_{0};
    rclcpp::Time start_{0, 0, RCL_ROS_TIME};

    bool have_command_{false};
    me130_interfaces::msg::MotorCommand command_;
    long long counts_{0};

    rclcpp::Subscription<me130_interfaces::msg::PendulumAngle>::SharedPtr angle_sub_;
    rclcpp::Subscription<me130_interfaces::msg::EncoderState>::SharedPtr encoder_sub_;
    rclcpp::Subscription<me130_interfaces::msg::MotorCommand>::SharedPtr command_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoggerNode>());
    rclcpp::shutdown();
    return 0;
}
